# Trainer model-family guide

The trainer supports two host-side correction-model families:

| Family | CLI value | Status | Best use |
|---|---|---|---|
| Gradient boosting | `gradient_boosting` | Default and currently implemented baseline | Small-to-moderate tabular datasets, feature-importance review, thesis baseline |
| Temporal 1-D CNN | `cnn_1d` | Experimental comparison path | Larger labelled datasets where causal temporal context can be evaluated without leakage |

Neither family replaces the firmware DSP/funnel estimator. Both consume the
trainer's leakage-filtered, engineered feature matrix and predict HR and/or RR
corrections on the host.

Install `rvt-trainer[stats]` when the optional participant-random-intercept
distance×barrier mixed-effects table is required. Reports fail closed with an
explicit unavailable-dependency status when that research-only extra is not
installed; GBR training itself does not import it.

## Recommended order

1. Fix firmware publish coverage and validate the telemetry contract.
2. Collect at least three independent sessions with clean reference data.
3. Train and report the gradient-boosting baseline first.
4. Enable the CNN only after each requested target has enough valid endpoints.
5. Compare held-out-session accuracy, coverage, bias, and failure modes. Do not
   select a model from training loss alone.

The CNN path requires 500 valid endpoint windows per target by default. This is
a safety floor, not proof that 500 windows are sufficient for a final claim.
More independent subjects and sessions are preferable to many correlated rows
from one recording.

## Gradient boosting

Gradient boosting remains the default and does not require a new dependency:

```bash
python -m rvt_trainer train \
  --radar session-a/radar.csv session-b/radar.csv \
  --ref session-a/reference.csv session-b/reference.csv \
  --model-family gradient_boosting \
  --three-way-split \
  --loo-eval \
  --out model-gbr
```

Existing commands that omit `--model-family` continue to use gradient boosting.

## Experimental 1-D CNN

Install TensorFlow only in the research environment that will train or load the
CNN:

```bash
python -m pip install tensorflow
```

Then select the temporal family explicitly:

```bash
python -m rvt_trainer train \
  --radar session-a/radar.csv session-b/radar.csv session-c/radar.csv \
  --ref session-a/reference.csv session-b/reference.csv session-c/reference.csv \
  --model-family cnn_1d \
  --cnn-window-size 32 \
  --cnn-min-valid-windows 500 \
  --three-way-split \
  --loo-eval \
  --out model-cnn
```

CNN inputs are causal windows shaped as
`[examples, timesteps, feature_channels]`. Windows are left-padded at cold
start and never cross a contiguous `session_id` boundary. Per-feature
normalization is learned only from the training windows and stored with the
model.

`--allow-small-cnn-dataset` exists only for labelled feasibility/overfit
experiments. Results produced with that override must not be presented as a
validated accuracy comparison.

## Artifacts and claims

Both families retain the existing `model_hr.pkl`, `model_rr.pkl`, and
`preprocessor.pkl` artifact contract and signed manifest verification. The
preprocessor, feature manifest, progress file, and training summary record the
selected family and CNN policy.

The optional embedded TFLite surrogate is a separate teacher-distillation
experiment. Selecting `cnn_1d` does not mean CNN inference is deployed on the
ESP32-C6, and producing a TFLite artifact does not establish hardware
acceptance.

## Reproducibility and bundle contract

Treat a trained directory as a versioned bundle, not as three interchangeable
pickle files. A reproducible bundle records:

- the model family and trained targets;
- the exact ordered base and expanded feature names;
- the feature-engineering version, schema version, and schema hash;
- a canonical feature-contract hash;
- all model, sequence, and preprocessing configuration;
- the root seed and the target seed policy (`HR = seed`, `RR = seed + 1`);
- Python, operating-system, machine, and dependency versions;
- artifact SHA-256 values and, for controlled use, a verified signature; and
- the source commit and trainer release that produced the bundle.

Preprocessing parameters must be fitted on the training partition only.
Validation, early-stop, test, and LOSO-held-out participants may be transformed
with the fitted values but must never influence imputation, missing-indicator
selection, feature selection, or CNN normalization. The saved feature contract
is order-sensitive: inference must fail closed when feature membership, order,
schema hash, model family, or CNN window size differs.

A fixed seed is necessary but not sufficient for bit-identical CNN results.
Record dependency and hardware/runtime versions, request deterministic
TensorFlow operations, and disclose when a deterministic kernel is unavailable.
TensorFlow remains an optional dependency and must not be imported while
training, loading metadata for, or running the default gradient-boosting path.

## Deployment trade-off gate

| Gate | Gradient boosting | 1-D CNN |
|---|---|---|
| Host status | Supported CPU baseline | Experimental; TensorFlow required |
| Feature input | Ordered 2-D tabular rows | Ordered causal 3-D windows |
| Main advantage | Strong small-data baseline and explainable importances | Learns temporal patterns within a session |
| Main risk | Limited representation of temporal dynamics | Higher data, dependency, memory, and reproducibility cost |
| ESP32-C6 status | Not qualified | Not qualified |
| Required embedded evidence | Conversion/distillation, flash, RAM, latency, parity, physical accuracy | Integer quantization, operator support, tensor arena, flash, latency, parity, physical accuracy |

Do not select the CNN because its training loss is lower. Select a deployable
family only after participant-held-out comparison at the protocol distances and
barriers, with identical preprocessing rules and accuracy/statistical gates.

## v16.5.9 statistical comparison contract

The trainer retains one `outer_oof_predictions.csv` per leave-one-participant-
out run. It contains raw predictions alongside the declared causal
range/slew-limited outputs, fold identity, participant/session/trial/condition
metadata, and a file hash. The statistical CLI consumes this artifact with the
versioned `quality/statistical-analysis-plan.json` and can emit JSON, flat CSV,
and LaTeX table outputs.

For confirmatory results, 30-second windows at 5-second stride are reduced to a
trial only when at least 15 windows are valid; a participant-condition requires
at least two valid trials. The 1.0 m/no-cardboard RR TOST (±2 breaths/minute,
alpha 0.05, 90% CI) is primary. The five remaining conditions are secondary
and use Holm adjustment. RMSE/MAE and agreement are participant-balanced, and
coverage/non-output denominators remain explicit. Fewer than 19 independent
primary estimates is reported as inconclusive. These rules do not authorize
CNN inference on the ESP32-C6.
