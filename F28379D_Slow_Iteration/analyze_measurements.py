"""Calculate fundamental RMS, THD through order 20 and residual RMS."""
from pathlib import Path
import csv
import json
import re
import numpy as np

ROOT = Path(__file__).resolve().parent
RATE = 2560
FREQUENCY = 50
CHANNELS = ('E1', 'E2', 'E3', 'I1', 'I2', 'I3')


def waveform(path):
    rows = []
    for line in path.read_text(encoding='utf-8-sig').splitlines():
        try:
            row = [float(v) for v in re.split(r'\s+', line.strip())]
        except ValueError:
            continue
        if len(row) >= 7:
            rows.append(row[:7])
    data = np.asarray(rows)
    if data.shape != (512, 7):
        raise ValueError(f'{path.name}: expected 512 six-channel samples')
    t = np.arange(512)/RATE
    x = data[:, 1:]
    basis = np.column_stack((np.ones(len(t)), np.sin(2*np.pi*FREQUENCY*t), np.cos(2*np.pi*FREQUENCY*t)))
    coefficients = np.linalg.lstsq(basis, x, rcond=None)[0]
    residual = x-basis@coefficients
    fundamental = np.hypot(coefficients[1], coefficients[2])/np.sqrt(2)
    spectrum = np.abs(np.fft.rfft(x, axis=0))*np.sqrt(2)/len(t)
    harmonics = spectrum[np.arange(1, 21)*10]
    return {'fundamental_rms': fundamental.tolist(),
            'total_rms': np.sqrt(np.mean(x*x, axis=0)).tolist(),
            'residual_rms': np.sqrt(np.mean(residual*residual, axis=0)).tolist(),
            'THD20_pct': (100*np.sqrt(np.sum(harmonics[1:]**2, axis=0))/fundamental).tolist(),
            'DC': coefficients[0].tolist()}


def analyzer_export(path):
    with path.open(encoding='utf-8-sig', newline='') as f:
        rows = list(csv.reader(f))
    values = np.asarray([[float(v) for v in row] for row in rows[2:] if row and row[0].strip()])
    if values.shape != (3, 43):
        raise ValueError(f'{path.name}: unexpected harmonic export shape {values.shape}')
    harmonics = np.sqrt(np.sum(values[:, 4:]**2, axis=1))
    ratio = 100*harmonics/values[:, 3]
    return {'exported_THD_pct': values[:, 0].tolist(),
            'exported_THD1_pct': values[:, 1].tolist(),
            'THD1_from_harmonic_columns_pct': ratio.tolist(),
            'THD_from_exported_THD1_pct': (100*values[:, 1]/np.sqrt(10000+values[:, 1]**2)).tolist()}


def main():
    output = ROOT/'output'
    output.mkdir(exist_ok=True)
    rounds = {str(n): waveform(ROOT/'Data'/f'Measured_Output_Round{n}.txt') for n in range(6)}
    instrument = {name: analyzer_export(ROOT/'Data/Harmonics'/(name+'.csv'))
                  for name in ('60V', '60VPERCENT', '60I', '60IPERCENT')}
    result = {'sample_rate_Hz': RATE, 'fundamental_Hz': FREQUENCY,
              'waveform_harmonic_orders': [2, 20], 'channels': CHANNELS,
              'units': ['V', 'V', 'V', 'A', 'A', 'A'], 'rounds': rounds,
              'instrument_exports': instrument,
              'hardware_current_reference_RMS_A': .253*.04/np.sqrt(2)}
    (output/'Measurements.json').write_text(json.dumps(result, indent=2), encoding='utf8')
    with (output/'Iteration results.csv').open('w', newline='', encoding='utf8') as f:
        writer = csv.writer(f)
        writer.writerow(['Round', 'Mean fundamental line voltage RMS (V)', 'Mean fundamental phase current RMS (mA)',
                         'Mean voltage THD20 (%)', 'Mean current THD20 (%)', 'Mean current residual RMS (mA)'])
        for n, r in rounds.items():
            writer.writerow([n, np.mean(r['fundamental_rms'][:3]), 1000*np.mean(r['fundamental_rms'][3:]),
                             np.mean(r['THD20_pct'][:3]), np.mean(r['THD20_pct'][3:]), 1000*np.mean(r['residual_rms'][3:])])
    print('Saved waveform and instrument calculations in '+str(output))


if __name__ == '__main__':
    main()
