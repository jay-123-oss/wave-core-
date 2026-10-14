//! Telemetry Stream & Real-Time Performance Monitor.
//!
//! Provides ring-buffered telemetry tracking, microsecond latency profiling,
//! and standard audit packet generation for verification benchmarks.

use crate::traits::TelemetryStream;
use crate::types::{ArbiterDecision, KinematicState3D};

/// In-memory fixed-size telemetry recorder.
#[derive(Clone, Debug)]
pub struct MemoryTelemetryLogger {
    /// Total cycles recorded.
    pub total_cycles: u64,
    /// Total cycles where safety arbiter intervened.
    pub intervention_cycles: u64,
    /// Minimum cycle latency observed (microseconds).
    pub min_latency_us: f32,
    /// Maximum cycle latency observed (microseconds).
    pub max_latency_us: f32,
    /// Cumulative latency sum for computing average.
    pub sum_latency_us: f64,
    /// Recent latency sample ring buffer (last 1000 samples).
    pub latency_samples: [f32; 1000],
    /// Ring buffer insertion index.
    pub sample_idx: usize,
}

impl Default for MemoryTelemetryLogger {
    fn default() -> Self {
        Self::new()
    }
}

impl MemoryTelemetryLogger {
    /// Creates a new telemetry logger with empty sample buffers.
    pub const fn new() -> Self {
        Self {
            total_cycles: 0,
            intervention_cycles: 0,
            min_latency_us: f32::MAX,
            max_latency_us: 0.0,
            sum_latency_us: 0.0,
            latency_samples: [0.0; 1000],
            sample_idx: 0,
        }
    }

    /// Computes arithmetic mean latency in microseconds.
    pub fn mean_latency_us(&self) -> f32 {
        if self.total_cycles == 0 {
            0.0
        } else {
            (self.sum_latency_us / self.total_cycles as f64) as f32
        }
    }

    /// Estimates 99th percentile latency across recent samples.
    pub fn p99_latency_us(&self) -> f32 {
        let n = if self.total_cycles < 1000 {
            self.sample_idx
        } else {
            1000
        };
        if n == 0 {
            return 0.0;
        }

        let mut sorted = [0.0f32; 1000];
        sorted[..n].copy_from_slice(&self.latency_samples[..n]);
        // Simple insertion sort for `#![no_std]` compatibility
        for i in 1..n {
            let mut j = i;
            while j > 0 && sorted[j - 1] > sorted[j] {
                sorted.swap(j - 1, j);
                j -= 1;
            }
        }
        let p99_idx = (n as f32 * 0.99) as usize;
        sorted[p99_idx.min(n - 1)]
    }
}

impl TelemetryStream for MemoryTelemetryLogger {
    fn log_cycle(&mut self, decision: &ArbiterDecision, _kinematics: &KinematicState3D) {
        self.total_cycles += 1;
        if decision.is_intervening {
            self.intervention_cycles += 1;
        }

        let lat = decision.latency_us;
        if lat < self.min_latency_us {
            self.min_latency_us = lat;
        }
        if lat > self.max_latency_us {
            self.max_latency_us = lat;
        }
        self.sum_latency_us += lat as f64;

        self.latency_samples[self.sample_idx] = lat;
        self.sample_idx = (self.sample_idx + 1) % 1000;
    }

    fn format_audit_packet(&self, decision: &ArbiterDecision) -> [u8; 128] {
        let mut packet = [0u8; 128];
        // Standardized binary header: 0x57415645 ("WAVE")
        packet[0] = 0x57;
        packet[1] = 0x41;
        packet[2] = 0x56;
        packet[3] = 0x45;

        // Action and flight mode
        packet[4] = decision.action as u8;
        packet[5] = decision.flight_mode as u8;
        packet[6] = if decision.is_intervening { 1 } else { 0 };
        packet[7] = if decision.anomaly_alert { 1 } else { 0 };

        // Serialized velocity (cmd_vx, cmd_vy, cmd_vz) as IEEE-754 LE
        let vx = decision.commanded_velocity.x.to_le_bytes();
        let vy = decision.commanded_velocity.y.to_le_bytes();
        let vz = decision.commanded_velocity.z.to_le_bytes();
        let cl = decision.min_clearance_m.to_le_bytes();
        let lat = decision.latency_us.to_le_bytes();

        packet[8..12].copy_from_slice(&vx);
        packet[12..16].copy_from_slice(&vy);
        packet[16..20].copy_from_slice(&vz);
        packet[20..24].copy_from_slice(&cl);
        packet[24..28].copy_from_slice(&lat);

        packet
    }

    fn statistics(&self) -> (u64, u64) {
        (self.total_cycles, self.intervention_cycles)
    }

    fn flush(&mut self) {
        // In-memory logger maintains real-time records
    }
}
