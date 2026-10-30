"""Minimal raw-stereo-video -> AnyHand -> SCH v2.1 temporal MANO entry point."""
import argparse
import json
import time
import traceback
from pathlib import Path

from ego_hand_benchmark.core.io import read_json, sha256, write_json
from ego_hand_benchmark.experiments import PROJECT_ROOT, resolve_output


def atomic_json(path, data):
    path = Path(path)
    temporary = path.with_suffix('.tmp.json')
    write_json(temporary, data)
    temporary.replace(path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True, help='Stereo video / calibration / timestamps JSON')
    parser.add_argument('--output', help='New run name')
    args = parser.parse_args()
    inputs = read_json(args.config)
    allowed = {'left_video', 'right_video', 'calibration', 'timestamps_left', 'timestamps_right',
               'timestamp_clock', 'sync_source', 'max_skew_ms'}
    if set(inputs) != allowed:
        raise ValueError(f'Input config must contain exactly: {sorted(allowed)}')
    settings = read_json(PROJECT_ROOT / 'configs/sch_v21_temporal.json')
    settings['optimizer']['mano_path'] = str(PROJECT_ROOT / 'WiLoR/mano_data/MANO_RIGHT.pkl')
    settings['max_skew_ms'] = inputs['max_skew_ms']
    output = resolve_output(args.output, family='sch_v21_temporal')
    output.mkdir(parents=True, exist_ok=False)
    state = dict(schema='sch_v2_video_input_v1', pipeline='sch-v2.1-hand', status='running',
                 configuration={'stage': 'mano_temporal', **inputs},
                 effective_reconstruction_configuration=settings, started_at=time.time(),
                 provenance=read_json(PROJECT_ROOT / 'PROVENANCE.json'))

    def progress(value):
        state['progress'] = value
        atomic_json(output / 'run.json', state)
        print(json.dumps(value), flush=True)

    progress({'stage': 'check_inputs'})
    try:
        state['input_sha256'] = {key: sha256(Path(inputs[key])) for key in
                                ('left_video', 'right_video', 'calibration', 'timestamps_left', 'timestamps_right')}
        from handpose_pipeline.pipelines.sch_v2_hand.video import prepare_video_inputs, run_video_inference
        from handpose_pipeline.pipelines.sch_v2_hand.pipeline import METHODS_V21, run_pipeline
        prepared = prepare_video_inputs(**inputs, cache_root=PROJECT_ROOT / 'data/derived/video', progress=progress)
        write_json(output / 'prepared.json', prepared)
        predictions = run_video_inference(prepared, output / 'inference', backend='anyhand_wilor', progress=progress)
        state['predictions'] = predictions
        state['reconstruction_predictions'] = run_pipeline(
            read_json(prepared['sequences']['left']['path']),
            read_json(prepared['sequences']['right']['path']),
            predictions['left'], predictions['right'], prepared['T_left_right'],
            output / 'reconstruction', stage='mano_temporal', config=settings,
            progress=progress, methods=METHODS_V21)
        state.update(status='complete', finished_at=time.time())
        atomic_json(output / 'run.json', state)
        print(f'完成：{output}')
    except BaseException:
        state.update(status='failed', error=traceback.format_exc(), finished_at=time.time())
        atomic_json(output / 'run.json', state)
        raise


if __name__ == '__main__':
    main()
