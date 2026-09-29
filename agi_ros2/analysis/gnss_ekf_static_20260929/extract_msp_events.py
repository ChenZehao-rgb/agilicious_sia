#!/usr/bin/env python3
"""Extract MSP events using embedded schemas and validate stored MCAP CRCs.

Usage with the diagnostic dependency installation:
    PYTHONPATH=/tmp/mcap-analysis-20260929 python3 extract_msp_events.py

Requires mcap and mcap-ros2-support. Absolute ROS/log/publish timestamps remain
integer nanoseconds; steady timestamps remain their original float64 values.
"""
import argparse
import csv
import gzip
import json
from pathlib import Path

from mcap.reader import NonSeekingReader, make_reader
from mcap_ros2.decoder import DecoderFactory


BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[2]
DEFAULT_BAG = ROOT / "bags/hardware_20260929_112215_827325"
TOPIC = "/msp/events"
FIELDS = ["log_ns", "publish_ns", "header.stamp_ns", "request_stamp_ns", "sequence",
          "bag_s", "stamp_s", "steady_time", "event", "code", "errors", "latency_seconds",
          "request_stamp_s", "request_steady_time", "request_name", "session_id", "payload"]


def nanoseconds(stamp):
    return int(stamp.sec) * 1000000000 + int(stamp.nanosec)


def extract(bag_directory, output_directory):
    paths = sorted(bag_directory.glob("*.mcap"))
    if not paths:
        raise FileNotFoundError(f"No MCAP files in {bag_directory}")
    starts = []
    expected = 0
    for path in paths:
        with path.open("rb") as stream:
            summary = make_reader(stream, validate_crcs=True).get_summary()
            if summary is None or summary.statistics is None:
                raise ValueError(f"MCAP summary/statistics required: {path}")
            starts.append(summary.statistics.message_start_time)
            expected += sum(count for channel_id, count in summary.statistics.channel_message_counts.items()
                            if summary.channels[channel_id].topic == TOPIC)
    start_ns = min(starts)
    output_directory.mkdir(parents=True, exist_ok=True)
    destination = output_directory / "msp_events_clock_audit.csv.gz"
    temporary = destination.with_suffix(destination.suffix + ".tmp")
    count = 0
    schemas = set()
    try:
        with gzip.open(temporary, "wt", newline="") as output:
            writer = csv.DictWriter(output, fieldnames=FIELDS)
            writer.writeheader()
            for path in paths:
                with path.open("rb") as stream:
                    # Sequential reading checks every stored chunk CRC and, when
                    # present, the data-section CRC, including unselected topics.
                    reader = NonSeekingReader(stream, validate_crcs=True,
                                              decoder_factories=[DecoderFactory()])
                    for schema, channel, record, message in reader.iter_decoded_messages(
                            topics=[TOPIC], log_time_order=False):
                        if schema is None or not schema.data or channel.message_encoding != "cdr":
                            raise ValueError("Missing embedded ROS2 schema or unsupported message encoding")
                        schemas.add(schema.name)
                        stamp_ns = nanoseconds(message.header.stamp)
                        request_ns = nanoseconds(message.request_stamp)
                        writer.writerow(dict(log_ns=record.log_time, publish_ns=record.publish_time,
                                             **{"header.stamp_ns": stamp_ns}, request_stamp_ns=request_ns,
                                             sequence=record.sequence,
                                             bag_s=(record.log_time - start_ns) / 1e9,
                                             stamp_s=(stamp_ns - start_ns) / 1e9,
                                             steady_time=message.steady_time, event=message.event,
                                             code=message.code, errors=message.errors,
                                             latency_seconds=message.latency_seconds,
                                             request_stamp_s=(request_ns - start_ns) / 1e9,
                                             request_steady_time=message.request_steady_time,
                                             request_name=message.request_name, session_id=message.session_id,
                                             payload=list(message.payload)))
                        count += 1
        if count != expected:
            raise ValueError(f"MSP event count mismatch: extracted={count}, MCAP summary={expected}")
        temporary.replace(destination)
    except BaseException:
        temporary.unlink(missing_ok=True)
        raise
    return dict(output=str(destination), start_ns=start_ns, extracted_count=count,
                mcap_summary_expected_count=expected, embedded_schemas=sorted(schemas),
                stored_crcs_validated=True, original_integer_timestamps_preserved=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag-directory", type=Path, default=DEFAULT_BAG)
    parser.add_argument("--output-directory", type=Path)
    args = parser.parse_args()
    output_directory = args.output_directory or BASE / args.bag_directory.name
    print(json.dumps(extract(args.bag_directory, output_directory), indent=2))


if __name__ == "__main__":
    main()
