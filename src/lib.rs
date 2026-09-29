//! SIH26123 — Edge-AI based distributed fleet coordination for AMRs.
//!
//! All of `docs/BUILD_PLAN.md`'s items (1-31) are done, including the optional item 29
//! (CBS baseline). `docs/FINAL_REPORT.md` compiles the real findings and checks
//! requirements compliance line-by-line against `PS_AND_ARCHITECTURE.md`. Decisions
//! 12-15 are closed; Decision 3 (degradation thresholds) is updated with real sweep
//! data but stays provisional — see `docs/FINAL_REPORT.md` §3 for what's still open.

pub mod clock_driver;
pub mod config;
pub mod dashboard;
pub mod protocol;
pub mod robot;
pub mod world;
