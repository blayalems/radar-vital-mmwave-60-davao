# Trainer model-family guide

The trainer supports two host-side correction-model families:

| Family | CLI value | Status | Best use |
|---|---|---|---|
| Gradient boosting | `gradient_boosting` | Default and currently implemented baseline | Small-to-moderate tabular datasets, feature-importance review, thesis baseline |
| Temporal 1-D CNN | `cnn_1d` | Experimental comparison path | Larger labelled datasets where causal temporal context can be evaluated without leakage |

Neither family replaces the firmware DSP/funnel estimator. Both consume the
trainer's leakage-filtered, engineered feature matrix and predict HR and/or RR
corrections on the host.

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
