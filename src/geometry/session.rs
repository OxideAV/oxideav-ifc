//! Per-model derived state shared by one tessellation run.
//!
//! Evaluating a model asks the same model-level questions over and
//! over: the plane-angle unit (once per trimmed conic), the world
//! transform of a placement chain (once per product, though thousands
//! of products hang off one storey placement) and the mesh of a shared
//! `IfcRepresentationMap` source (once per `IfcMappedItem` instance).
//! A [`Session`] answers each of them once.
//!
//! The session is installed on the current thread for the duration of
//! a borrow of the [`StepFile`] (so the cached answers cannot outlive
//! or disagree with the file) and found again by the geometry code
//! through the file's address. Every public entry point of the
//! geometry module opens one when none is active; the scene decoder
//! opens one for the whole model and lends it to its worker threads.

use std::cell::RefCell;
use std::collections::HashMap;
use std::marker::PhantomData;
use std::sync::{Arc, Mutex, OnceLock};

use super::{GeometryError, Transform, TriMesh};
use crate::parser::StepFile;

/// The cached answers of one model.
#[derive(Debug, Default)]
pub(crate) struct Shared {
    /// Address of the `StepFile` the answers belong to.
    key: usize,
    angle_scale: OnceLock<Option<f64>>,
    placements: Mutex<HashMap<u64, Transform>>,
    representations: Mutex<HashMap<u64, Arc<Result<TriMesh, GeometryError>>>>,
}

thread_local! {
    static ACTIVE: RefCell<Option<Arc<Shared>>> = const { RefCell::new(None) };
}

fn key_of(step: &StepFile) -> usize {
    core::ptr::from_ref(step) as usize
}

/// A session installed on the current thread; the previous one is
/// restored on drop. Tied to the borrow of the file it describes.
pub(crate) struct Session<'a> {
    previous: Option<Arc<Shared>>,
    installed: bool,
    // Not `Send`: the guard restores the thread-local it changed.
    _step: PhantomData<(&'a StepFile, *const ())>,
}

impl<'a> Session<'a> {
    /// Make sure a session for `step` is active on this thread: a new
    /// one unless the active one already describes this file.
    pub(crate) fn enter(step: &'a StepFile) -> Self {
        let key = key_of(step);
        ACTIVE.with(|active| {
            let mut active = active.borrow_mut();
            if active.as_ref().is_some_and(|s| s.key == key) {
                return Session {
                    previous: None,
                    installed: false,
                    _step: PhantomData,
                };
            }
            let previous = active.replace(Arc::new(Shared {
                key,
                ..Shared::default()
            }));
            Session {
                previous,
                installed: true,
                _step: PhantomData,
            }
        })
    }

    /// Install an existing session's state on this (worker) thread.
    #[cfg_attr(not(feature = "registry"), allow(dead_code))]
    pub(crate) fn join(_step: &'a StepFile, shared: &Arc<Shared>) -> Self {
        let previous = ACTIVE.with(|active| active.borrow_mut().replace(Arc::clone(shared)));
        Session {
            previous,
            installed: true,
            _step: PhantomData,
        }
    }

    /// The state of the session active on this thread, to hand to
    /// worker threads.
    #[cfg_attr(not(feature = "registry"), allow(dead_code))]
    pub(crate) fn shared(&self) -> Arc<Shared> {
        ACTIVE
            .with(|active| active.borrow().clone())
            .expect("a session is active while its guard lives")
    }
}

impl Drop for Session<'_> {
    fn drop(&mut self) {
        if self.installed {
            let previous = self.previous.take();
            ACTIVE.with(|active| *active.borrow_mut() = previous);
        }
    }
}

fn current(step: &StepFile) -> Option<Arc<Shared>> {
    let key = key_of(step);
    ACTIVE.with(|active| active.borrow().as_ref().filter(|s| s.key == key).cloned())
}

/// Radians per model plane-angle unit (1 when the model declares none).
pub(crate) fn angle_scale(step: &StepFile) -> f64 {
    let resolve = || crate::schema::plane_angle_unit_scale(step);
    match current(step) {
        Some(shared) => *shared.angle_scale.get_or_init(resolve),
        None => resolve(),
    }
    .unwrap_or(1.0)
}

/// The cached world transform of placement `id`, if already resolved.
pub(crate) fn placement(step: &StepFile, id: u64) -> Option<Transform> {
    let shared = current(step)?;
    let map = shared.placements.lock().ok()?;
    map.get(&id).copied()
}

/// Remember the world transform of placement `id`.
pub(crate) fn remember_placement(step: &StepFile, id: u64, world: Transform) {
    if let Some(shared) = current(step)
        && let Ok(mut map) = shared.placements.lock()
    {
        map.insert(id, world);
    }
}

/// The mesh of shared source representation `id`, computed by `build`
/// on first use. Two threads racing on one representation both compute
/// it (the result is deterministic) and the first one stored wins.
pub(crate) fn representation(
    step: &StepFile,
    id: u64,
    build: impl FnOnce() -> Result<TriMesh, GeometryError>,
) -> Arc<Result<TriMesh, GeometryError>> {
    let Some(shared) = current(step) else {
        return Arc::new(build());
    };
    if let Ok(map) = shared.representations.lock()
        && let Some(hit) = map.get(&id)
    {
        return Arc::clone(hit);
    }
    let built = Arc::new(build());
    match shared.representations.lock() {
        Ok(mut map) => Arc::clone(map.entry(id).or_insert(built)),
        Err(_) => built,
    }
}
