import re
import json

def parse_avg_line(line):
    parts = re.split(r'\s+', line)
    return {
        'usr': float(parts[3]),
        'system': float(parts[4]),
        'cpu': float(parts[7]),
    }

def extract_aggregate_avg(file_path):
    with open(file_path, 'r') as f:
        usr = 0.0
        system = 0.0
        cpu = 0.0

        for line in f:
            if not line.startswith('Average'):
                continue
            if line.endswith('Command\n'):
                continue
            d = parse_avg_line(line)
            if d['cpu'] < 1.0:
                # Consider this line represents the daemon process
                continue

            usr += d['usr']
            system += d['system']
            cpu += d['cpu']

        return {
            'usr': usr,
            'system': system,
            'cpu': cpu,
        }

def main():
    with open('./pidstat_results/index.json', 'r') as f:
        measurements = json.load(f)

    for i, m in enumerate(measurements, 1):
        print(
            f"------------------Setting {i:>2}------------------\n"
            f"composition_pattern: {m['composition_pattern']}\n"
            f"num_topics: {m['num_topics']}\n"
            f"use_multithreaded_executor: {m['use_multithreaded_executor']}\n"
            f"talker_ros2_thread_count: {m['talker_ros2_thread_count']}\n"
            f"talker_agnocast_thread_count: {m['talker_agnocast_thread_count']}\n"
            f"listener_ros2_thread_count: {m['listener_ros2_thread_count']}\n"
            f"listener_agnocast_thread_count: {m['listener_agnocast_thread_count']}\n"
            f"timer_interval_ms: {m['timer_interval_ms']}\n"
        )

        talker = extract_aggregate_avg(f"./pidstat_results/{m['talker_path']}")
        listener = extract_aggregate_avg(f"./pidstat_results/{m['listener_path']}")

        print(
            f"            %usr %system    %CPU\n"
            f"talker   {talker['usr']:>7.2f} {talker['system']:>7.2f} {talker['cpu']:>7.2f}\n"
            f"listener {listener['usr']:>7.2f} {listener['system']:>7.2f} {listener['cpu']:>7.2f}\n"
            f"----------------------------------------------\n"
        )


if __name__ == "__main__":
    main()
