//! Parse an arbitrary byte string as a STEP physical file and run the
//! Phase-3 geometry extractor over every instance it contains: the
//! tessellators (face sets, faceted / advanced Breps, swept solids,
//! directrix sweeps, Booleans, mapped items, curved-face trimming) must
//! never panic, loop or allocate without bound on hostile input.

#![no_main]

use libfuzzer_sys::fuzz_target;
use oxideav_ifc::{parse_step_with_limits, tessellate_item, StepLimits};

fuzz_target!(|data: &[u8]| {
    let limits = StepLimits {
        max_input_len: 64 * 1024,
        max_instances: 4096,
        max_depth: 16,
        max_string_len: 1024,
    };
    let Ok(step) = parse_step_with_limits(data, &limits) else {
        return;
    };
    let mut ids: Vec<u64> = step.instances.keys().copied().collect();
    ids.sort_unstable();
    for id in ids {
        // Results are irrelevant; only the absence of panics / runaway
        // work matters.
        let _ = tessellate_item(&step, id);
    }
});
