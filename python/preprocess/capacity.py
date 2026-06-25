import scipy.io as sio
import os
import csv

try:
    print("[Load] Loading dataset...")
    dataset_path = './dataset'
    battery_files = sorted([f for f in os.listdir(dataset_path) if f.endswith('.mat')])
    print("[Success] Successfully loaded dataset!")
except Exception as e:
    print(f'[Error] {e} while loading dataset.')
    exit(1)

try:
    print("[Load] Creating output directory...")
    os.makedirs('./output/capacity', exist_ok=True)
    print("[Success] Successfully created output directory!")
except Exception as e:
    print(f'[Error] {e} while creating output directory.')
    exit(1)

for battery_file in battery_files:
    try:
        battery_name = battery_file.replace('.mat', '')
        print(f"[Load] Processing {battery_name}...")
        mat_data = sio.loadmat(os.path.join(dataset_path, battery_file))
    except Exception as e:
        print(f'[Error] {e} while loading {battery_file}.')
        continue

    try:
        battery = mat_data[battery_name]
        cycles = battery['cycle'][0, 0]
    except Exception as e:
        print(f'[Error] {e} while parsing {battery_name}.')
        continue

    try:
        discharge_idx = 0
        with open(f'./output/capacity/{battery_name}_capacity.csv', 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(['cycle', 'capacity'])
            for i in range(cycles.shape[1]):
                try:
                    cycle = cycles[0, i]
                    if cycle['type'][0] != 'discharge':
                        continue
                    cycle_data = cycle['data'][0, 0]
                    capacity = float(cycle_data['Capacity'].flatten()[0])
                    writer.writerow([discharge_idx, capacity])
                    discharge_idx += 1
                except Exception as e:
                    print(f'[Error] {e} while processing cycle {i} of {battery_name}.')
                    continue
        print(f"[Success] {battery_name} -> {discharge_idx} discharge cycles saved!")
    except Exception as e:
        print(f'[Error] {e} while writing CSV for {battery_name}.')
        continue

print('========== Done!! Saved CSV in ./output/capacity directory! ==========')
