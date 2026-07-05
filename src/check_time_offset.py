#!/usr/bin/env python3
import argparse
import bisect
import statistics
import threading

import rospy
import rostopic


def parse_args():
    parser = argparse.ArgumentParser(
        description="Estimate whether two ROS topics have a fixed header.stamp time offset."
    )
    parser.add_argument("--livox-topic", default="/livox/lidar")
    parser.add_argument("--radar-topic", default="/radar_enhanced_pcl")
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--max-lag", type=int, default=5)
    parser.add_argument("--report-period", type=float, default=2.0)
    parser.add_argument("--fixed-std-threshold", type=float, default=0.02)
    parser.add_argument(
        "--gap-threshold",
        type=float,
        default=0.2,
        help="Header stamp gap threshold in seconds for detecting stream dropouts.",
    )
    return parser.parse_args(rospy.myargv()[1:])


def get_topic_msg_class(topic):
    msg_class, real_topic, _ = rostopic.get_topic_class(topic, blocking=True)
    if msg_class is None:
        raise RuntimeError("Cannot resolve message type for {}".format(topic))
    return msg_class, real_topic


def stamp_from_msg(msg):
    if not hasattr(msg, "header"):
        raise AttributeError("message has no header")
    return msg.header.stamp.to_sec()


def summarize(values):
    if not values:
        return None
    values_sorted = sorted(values)
    return {
        "count": len(values_sorted),
        "mean": statistics.mean(values_sorted),
        "median": statistics.median(values_sorted),
        "std": statistics.pstdev(values_sorted) if len(values_sorted) > 1 else 0.0,
        "min": values_sorted[0],
        "max": values_sorted[-1],
    }


def format_summary(name, summary):
    if summary is None:
        return "{}: no data".format(name)
    return (
        "{}: n={} mean={:+.6f}s median={:+.6f}s std={:.6f}s "
        "min={:+.6f}s max={:+.6f}s"
    ).format(
        name,
        summary["count"],
        summary["mean"],
        summary["median"],
        summary["std"],
        summary["min"],
        summary["max"],
    )


def consecutive_intervals(stamps):
    stamps_sorted = sorted(stamps)
    return [
        stamps_sorted[i] - stamps_sorted[i - 1]
        for i in range(1, len(stamps_sorted))
    ]


def format_period_summary(name, stamps, gap_threshold):
    intervals = consecutive_intervals(stamps)
    summary = summarize(intervals)
    if summary is None:
        return "{} period: no data".format(name)

    gaps = [value for value in intervals if value > gap_threshold]
    line = (
        "{} period: n={} mean={:.6f}s median={:.6f}s std={:.6f}s "
        "min={:.6f}s max={:.6f}s gaps>{:.3f}s={}"
    ).format(
        name,
        summary["count"],
        summary["mean"],
        summary["median"],
        summary["std"],
        summary["min"],
        summary["max"],
        gap_threshold,
        len(gaps),
    )
    if gaps:
        line += " largest_gaps=" + ",".join(
            "{:.3f}".format(value) for value in sorted(gaps, reverse=True)[:5]
        )
    return line


def overlap_window(livox, radar):
    if not livox or not radar:
        return None
    start = max(livox[0], radar[0])
    end = min(livox[-1], radar[-1])
    if start > end:
        return None
    return start, end


class OffsetEstimator:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        self.livox_stamps = []
        self.radar_stamps = []
        self.last_warn = {"livox": False, "radar": False}

    def livox_cb(self, msg):
        self._append_stamp("livox", msg)

    def radar_cb(self, msg):
        self._append_stamp("radar", msg)

    def _append_stamp(self, name, msg):
        try:
            stamp = stamp_from_msg(msg)
        except AttributeError:
            if not self.last_warn[name]:
                rospy.logerr("%s message does not contain header.stamp", name)
                self.last_warn[name] = True
            return

        with self.lock:
            stamps = self.livox_stamps if name == "livox" else self.radar_stamps
            stamps.append(stamp)
            if len(stamps) > self.args.samples:
                del stamps[: len(stamps) - self.args.samples]

    def report(self, _event):
        with self.lock:
            livox = list(self.livox_stamps)
            radar = list(self.radar_stamps)

        if len(livox) < 5 or len(radar) < 5:
            rospy.loginfo(
                "waiting for data... livox=%d radar=%d", len(livox), len(radar)
            )
            return

        livox_sorted = sorted(livox)
        radar_sorted = sorted(radar)
        overlap = overlap_window(livox_sorted, radar_sorted)
        best_lag, best_values, best_summary = self._best_lag_summary(
            livox_sorted, radar_sorted
        )
        nearest_all_summary = summarize(self._nearest_deltas(livox_sorted, radar_sorted))
        nearest_overlap_values, overlap_info = self._nearest_overlap_deltas(
            livox_sorted, radar_sorted, overlap
        )
        nearest_overlap_summary = summarize(nearest_overlap_values)
        livox_period_line = format_period_summary(
            "livox", livox_sorted, self.args.gap_threshold
        )
        radar_period_line = format_period_summary(
            "radar", radar_sorted, self.args.gap_threshold
        )

        rospy.loginfo(
            "\n%s\n%s",
            "-" * 72,
            self._format_report(
                best_lag,
                best_summary,
                nearest_all_summary,
                nearest_overlap_summary,
                overlap_info,
                livox_period_line,
                radar_period_line,
            ),
        )

    def _best_lag_summary(self, livox, radar):
        best_lag = 0
        best_values = []
        best_summary = None
        min_count = max(5, min(len(livox), len(radar)) // 3)

        for lag in range(-self.args.max_lag, self.args.max_lag + 1):
            values = []
            for i, livox_stamp in enumerate(livox):
                radar_index = i + lag
                if 0 <= radar_index < len(radar):
                    values.append(radar[radar_index] - livox_stamp)
            if len(values) < min_count:
                continue
            summary = summarize(values)
            if best_summary is None or summary["std"] < best_summary["std"]:
                best_lag = lag
                best_values = values
                best_summary = summary

        return best_lag, best_values, best_summary

    def _nearest_deltas(self, livox, radar):
        values = []
        for livox_stamp in livox:
            insert_at = bisect.bisect_left(radar, livox_stamp)
            candidates = []
            if insert_at < len(radar):
                candidates.append(radar[insert_at])
            if insert_at > 0:
                candidates.append(radar[insert_at - 1])
            if candidates:
                nearest = min(candidates, key=lambda stamp: abs(stamp - livox_stamp))
                values.append(nearest - livox_stamp)
        return values

    def _nearest_overlap_deltas(self, livox, radar, overlap):
        if overlap is None:
            return [], "overlap: none"

        start, end = overlap
        livox_overlap = [stamp for stamp in livox if start <= stamp <= end]
        radar_overlap = [stamp for stamp in radar if start <= stamp <= end]
        values = self._nearest_deltas(livox_overlap, radar_overlap)
        overlap_info = (
            "overlap: start={:.6f} end={:.6f} duration={:.3f}s "
            "livox_used={}/{} radar_used={}/{}"
        ).format(
            start,
            end,
            end - start,
            len(livox_overlap),
            len(livox),
            len(radar_overlap),
            len(radar),
        )
        return values, overlap_info

    def _format_report(
        self,
        best_lag,
        best_summary,
        nearest_all_summary,
        nearest_overlap_summary,
        overlap_info,
        livox_period_line,
        radar_period_line,
    ):
        if best_summary is None:
            return "not enough paired samples"

        fixed = best_summary["std"] <= self.args.fixed_std_threshold
        suggested_offset = -best_summary["median"]
        lines = [
            "samples: livox={} radar={}".format(
                len(self.livox_stamps), len(self.radar_stamps)
            ),
            format_summary("index/lag aligned dt = radar_stamp - livox_stamp", best_summary),
            "best_lag: radar_index = livox_index + {}".format(best_lag),
            format_summary("nearest-frame all dt = radar_stamp - livox_stamp", nearest_all_summary),
            overlap_info,
            format_summary("nearest-frame overlap dt = radar_stamp - livox_stamp", nearest_overlap_summary),
            "fixed_offset_like: {} (std threshold {:.3f}s)".format(
                "YES" if fixed else "NO", self.args.fixed_std_threshold
            ),
            "suggested sync/radar_time_offset: {:+.6f}s".format(suggested_offset),
            livox_period_line,
            radar_period_line,
        ]
        return "\n".join(lines)


def main():
    args = parse_args()
    rospy.init_node("check_time_offset", anonymous=True)

    try:
        livox_class, livox_topic = get_topic_msg_class(args.livox_topic)
        radar_class, radar_topic = get_topic_msg_class(args.radar_topic)
    except Exception as exc:
        rospy.logerr(
            "Failed to resolve topic message classes: %s\n"
            "Please source your workspace first, for example:\n"
            "  source /home/SENSETIME/wenkai/fast_lio/devel/setup.bash",
            exc,
        )
        return

    estimator = OffsetEstimator(args)
    rospy.Subscriber(livox_topic, livox_class, estimator.livox_cb, queue_size=200)
    rospy.Subscriber(radar_topic, radar_class, estimator.radar_cb, queue_size=200)
    rospy.Timer(rospy.Duration(args.report_period), estimator.report)

    rospy.loginfo("livox topic: %s", livox_topic)
    rospy.loginfo("radar topic: %s", radar_topic)
    rospy.loginfo("collecting up to %d samples per topic...", args.samples)
    rospy.spin()


if __name__ == "__main__":
    main()
