#!/usr/bin/python3

import copy
import os
import argparse
import json
import glob
import statistics

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("input_path_expr")
arg_parser.add_argument("output_file_path")
args = arg_parser.parse_args()

tp_counts = []
fp_counts = []
fn_counts = []
precisions = []
recalls = []
f1_scores = []

for input_file_path in glob.glob(args.input_path_expr):
    input_file_basename = os.path.basename(input_file_path)

    with open(input_file_path, "r") as input_file:
        causal_discovery_json = json.load(input_file)

        convoy_head_id = causal_discovery_json["convoy_head_id"]
        convoy_tail_id = causal_discovery_json["convoy_tail_id"]
        independent_id = causal_discovery_json["independent_id"]

        causal_links = causal_discovery_json["causal_links"]

        tp_count = 0
        fp_count = 0
        fn_count = 0

        for cause_str in causal_links:
            cause_id = int(cause_str)
            effect_ids = causal_links[cause_str]
            if cause_id == convoy_head_id:
                if convoy_tail_id in effect_ids:
                    tp_count += 1
                    fp_count += len(effect_ids) - 1
                else:
                    fp_count += len(effect_ids)
                    fn_count += 1
            else:
                fp_count += len(effect_ids)

        if causal_links.get(str(convoy_head_id)) is None:
            fn_count += 1

        tp_counts.append(tp_count)
        fp_counts.append(fp_count)
        fn_counts.append(fn_count)

        if len(causal_links) == 0:
            precision = 0
        else:
            precision = tp_count / (tp_count + fp_count)

        recall = tp_count / (tp_count + fn_count)

        f1_score = (2 * tp_count) / (2 * tp_count + fp_count + fn_count)

        precisions.append(precision)
        recalls.append(recall)
        f1_scores.append(f1_score)

precision_mean = statistics.mean(precisions)
precision_stdev = statistics.stdev(precisions)

recall_mean = statistics.mean(recalls)
recall_stdev = statistics.stdev(recalls)

f1_score_mean = statistics.mean(f1_scores)
f1_score_stdev = statistics.stdev(f1_scores)

with open(args.output_file_path, "w") as output_file:
    performance_evaluation_json = {
        "precision": {
            "mean": precision_mean,
            "stdev": precision_stdev
        },
        "recall": {
            "mean": recall_mean,
            "stdev": recall_stdev
        },
        "f1_score": {
            "mean": f1_score_mean,
            "stdev": f1_score_stdev
        }
    }
    json.dump(performance_evaluation_json, output_file)
