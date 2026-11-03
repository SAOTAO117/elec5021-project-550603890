from pathlib import Path
import argparse
import json
import shutil
import subprocess
import make_replay
import analyze_losses

ROOT = Path(__file__).resolve().parent

def main():
    parser = argparse.ArgumentParser(description='EPC2218 full-cycle loss and sensitivity calculations.')
    parser.add_argument('--ngspice', help='Path to ngspice or ngspice_con executable')
    parser.add_argument('--prepare-only', action='store_true', help='Write and validate the replay inputs without simulating')
    args = parser.parse_args()
    method = ROOT.parent.name
    assert method in ('SPWM', 'SVPWM')
    output = ROOT/'output'
    output.mkdir(exist_ok=True)
    (output/'.spiceinit').write_text('set ngbehavior=lt\nset filetype=binary\nset num_threads=1\n')
    shutil.copy2(ROOT/'EPC2218Scaled.net', output/'EPC2218Scaled.net')
    make_replay.R = output
    make_replay.OLD = ROOT/'Inputs'
    analyze_losses.G = output
    executable = args.ngspice or shutil.which('ngspice_con') or shutil.which('ngspice')
    if not args.prepare_only and not executable:
        parser.error('ngspice was not found; provide --ngspice with the executable path.')
    cases = [('Nominal', 1e-6, .5e-6, 1e-4), ('Strict', 1e-6, .5e-6, 3e-5),
             ('Deadtime05', .5e-6, .5e-6, 1e-4), ('Deadtime20', 2e-6, .5e-6, 1e-4),
             ('Current1us', 1e-6, 1e-6, 1e-4)]
    results = {}
    for name, deadtime, spacing, tolerance in cases:
        tag = method+'_'+name
        make_replay.generate(method, deadtime, spacing, tag)
        circuit = output/(tag+'.cir')
        content = circuit.read_text().replace('.include EPC2218V5.net', '.include EPC2218Scaled.net')
        content = content.replace('reltol=1e-4', 'reltol='+format(tolerance, 'g'))
        circuit.write_text(content)
        if args.prepare_only:
            print('Prepared '+tag)
            continue
        subprocess.run([executable, '-b', '-r', tag+'.raw', '-o', tag+'.log', tag+'.cir'],
                       cwd=output, check=True)
        result = analyze_losses.analyze(tag)
        if not result['complete']:
            raise RuntimeError(tag+' did not finish the complete 20 ms analysis interval.')
        results[tag] = result
        print(tag+': '+format(result['device_W']*1000, '.3f')+' mW')
        (output/'losses.json').write_text(json.dumps(results, indent=2), encoding='utf8')

if __name__ == '__main__':
    main()
