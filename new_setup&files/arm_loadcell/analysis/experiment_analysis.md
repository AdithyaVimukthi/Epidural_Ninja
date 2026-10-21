# Needle insertion experiment analysis

Experiment 0 is excluded at the user's request. Original recordings are preserved.
Experiments 1–3 are needle insertion into silicone. Experiment 4 is needle blocked
by PLA bone. Experiment 5 involved PLA blockage, but the user reports that the
load-cell holding bracket broke into two parts. It is retained as a failure record
and excluded from the intact-setup comparison and repeatability assessment.

The threshold is 4,000,000 raw ADC counts, as shown in the saved graphs. The CSVs
contain no calibration, insertion depth/speed, or contact/failure annotations.

| Experiment | Condition | Samples | Recorded span (s) | Maximum raw counts | Minimum raw counts | Samples >4,000,000 | Estimated time above (s) |
|---|---|---:|---:|---:|---:|---:|---:|
| 1 | Silicone | 1108 | 12.397 | 1,962,936 | -1,672,143 | 0 | 0.000 |
| 2 | Silicone | 1379 | 15.459 | 2,158,665 | -1,631,277 | 0 | 0.000 |
| 3 | Silicone | 1926 | 21.625 | 4,014,013 | -2,291,164 | 3 | 0.036 |
| 4 | PLA bone blockage, no failure reported | 1337 | 15.024 | 4,347,106 | -172,799 | 144 | 1.628 |
| 5 | Bracket failure, diagnostic only | 930 | 10.453 | 4,351,517 | -52,000 | 36 | 0.406 |

Recorded span is last timestamp minus first timestamp. Time above threshold uses
linear interpolation between consecutive samples, summed across all above-threshold
intervals. It is not necessarily one continuous event. No smoothing, tare correction,
or calibration was applied.

## Intact-setup comparison: experiments 1–4

- Experiment 4 peaks at 4,347,106 counts, versus the highest silicone peak of
  4,014,013 in experiment 3: a difference of 333,093 counts, or 8.3% relative to
  that silicone peak. This is a raw-signal comparison, not calibrated force.
- The existing threshold flags experiment 4 and also silicone experiment 3.
  Experiment 3 crosses it for only three samples (about 0.036 s total), whereas
  experiment 4 spends about 1.628 s above it.
- A persistence rule could reject brief crossings like experiment 3's. It must
  use continuous exceedance duration and be evaluated on further trials, rather
  than inferred solely from total time above threshold.
- There is a peak-value gap between these observed silicone runs and experiment
  4. A threshold within that gap would separate these four records, but choosing
  it from the same records would not establish performance on new experiments.
- Only one PLA trial without a reported fixture failure remains. PLA repeatability
  cannot be assessed from this dataset. The earlier comparison of experiments
  4 and 5 as repeated intact PLA trials is withdrawn.
- Silicone records have substantial negative excursions after positive loading.
  Withdrawal, load reversal, or mounting behaviour are possible explanations;
  event timing is needed to distinguish them.

## Experiment 5: bracket failure

The signal reaches 4,351,517 counts at 4.624 s. Near 4.680 s, it falls by 2,461,322
counts between consecutive samples. The user's reported bracket break provides a
plausible mechanical explanation for the abrupt unloading. The exact fracture
instant was not recorded, so 4.680 s is a signal-change time, not a confirmed
fracture time. Neither this peak nor the threshold is a calibrated bracket
breaking load. Data after failure describe a changed fixture configuration and
must not be compared as ordinary intact-setup measurements.

## Data quality and next steps

Sampling is consistent at 88.87–89.30 readings/s for runs 1–5. Every interval is
11 or 12 ms, timestamps strictly increase, and no sample reaches the signed
24-bit ADC rails. Absence of ADC clipping does not demonstrate that the fixture
or load cell stayed within mechanical limits.

Repair or replace the bracket before repeating the PLA condition. Record a
confirmed unloaded baseline, consistent insertion depth and speed, needle type,
mounting conditions, and contact/withdrawal event times. Repeat both conditions
with an intact fixture. Calibrate with known loads to express physical force or
mass, then evaluate peak and continuous-threshold rules on independent trials.

## Source files

- recordings/experiment1_20261011_113022_308240.csv
- recordings/experiment2_20261011_113058_873938.csv
- recordings/experiment3_20261011_113131_181086.csv
- recordings/experiment4_20261011_113217_638139.csv
- recordings/experiment5_20261011_113305_681040.csv (failure record only)
