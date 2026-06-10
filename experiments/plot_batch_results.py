from __future__ import annotations

import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np


PARAM_COLUMNS = [
    "planner",
    "start_positions_mode",
    "terrain",
    "probability_map",
    "num_drones",
    "detection_radius",
    "time_budget",
]

METRIC_COLUMNS = [
    "coverage",
    "expected_target_detection_time",
    "total_distance",
    "coverage_per_distance",
    "total_waypoints",
    "best_fitness",
]

SERIES_PRIORITY = [
    "planner",
    "detection_radius",
    "num_drones",
    "time_budget",
    "start_positions_mode",
    "terrain",
    "probability_map",
]


def parse_value(value: str):
    if value is None:
        return None

    text = value.strip()

    if text == "":
        return None

    try:
        number = float(text)
    except ValueError:
        return text

    if number.is_integer():
        return int(number)

    return number


def is_number(value) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool)


def read_rows(csv_path: Path) -> list[dict]:
    with csv_path.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        return [
            {
                key: parse_value(value)
                for key, value in row.items()
            }
            for row in reader
        ]


def unique_values(rows: list[dict], column: str) -> list:
    values = {
        row.get(column)
        for row in rows
        if row.get(column) is not None
    }

    return sorted(
        values,
        key=lambda value: (
            0 if is_number(value) else 1,
            value if is_number(value) else str(value),
        ),
    )


def numeric_column(rows: list[dict], column: str) -> bool:
    values = [
        row.get(column)
        for row in rows
        if row.get(column) is not None
    ]

    return bool(values) and all(is_number(value) for value in values)


def aggregate(
    rows: list[dict],
    key_columns: list[str],
    metric: str,
) -> dict[tuple, float]:
    buckets = defaultdict(list)

    for row in rows:
        value = row.get(metric)

        if not is_number(value):
            continue

        key = tuple(row.get(column) for column in key_columns)
        buckets[key].append(float(value))

    return {
        key: float(np.mean(values))
        for key, values in buckets.items()
        if values
    }


def title_for(metric: str, x_column: str) -> str:
    return f"{metric.replace('_', ' ')} by {x_column.replace('_', ' ')}"


def choose_series_columns(
    varying_columns: list[str],
    x_column: str,
) -> list[str]:
    return [
        column
        for column in SERIES_PRIORITY
        if column in varying_columns and column != x_column
    ]


def series_label(columns: list[str], values: tuple) -> str:
    return ", ".join(
        f"{column}={value}"
        for column, value in zip(columns, values)
    )


def save_line_plot(
    rows: list[dict],
    output_dir: Path,
    metric: str,
    x_column: str,
    series_columns: list[str],
) -> Path | None:
    key_columns = [x_column] + series_columns

    data = aggregate(rows, key_columns, metric)

    if not data:
        return None

    plt.figure(figsize=(8, 5))

    if series_columns:
        series_values = sorted(
            {
                tuple(row.get(column) for column in series_columns)
                for row in rows
            },
            key=lambda values: tuple(str(value) for value in values),
        )

        for series_value in series_values:
            points = [
                (key[0], value)
                for key, value in data.items()
                if key[1:] == series_value
            ]

            if not points:
                continue

            points.sort(key=lambda item: item[0])
            x_values, y_values = zip(*points)
            plt.plot(
                x_values,
                y_values,
                marker="o",
                linewidth=2,
                label=series_label(
                    series_columns,
                    series_value,
                ),
            )

        plt.legend(fontsize=8)
    else:
        points = sorted(
            ((key[0], value) for key, value in data.items()),
            key=lambda item: item[0],
        )
        x_values, y_values = zip(*points)
        plt.plot(x_values, y_values, marker="o", linewidth=2)

    plt.title(title_for(metric, x_column))
    plt.xlabel(x_column.replace("_", " "))
    plt.ylabel(metric.replace("_", " "))
    plt.grid(True, alpha=0.3)
    plt.tight_layout()

    suffix = ""
    if series_columns:
        suffix = "_by_" + "_and_".join(series_columns)

    output_path = (
        output_dir
        / f"line_{metric}_by_{x_column}{suffix}.png"
    )
    plt.savefig(output_path, dpi=160)
    plt.close()

    return output_path


def save_bar_plot(
    rows: list[dict],
    output_dir: Path,
    metric: str,
    x_column: str,
    series_columns: list[str],
) -> Path | None:
    key_columns = [x_column] + series_columns

    data = aggregate(rows, key_columns, metric)

    if not data:
        return None

    x_values = unique_values(rows, x_column)
    x_indexes = np.arange(len(x_values))

    plt.figure(figsize=(8, 5))

    if series_columns:
        series_values = sorted(
            {
                tuple(row.get(column) for column in series_columns)
                for row in rows
            },
            key=lambda values: tuple(str(value) for value in values),
        )
        width = 0.8 / max(1, len(series_values))

        for index, series_value in enumerate(series_values):
            y_values = [
                data.get((x_value, *series_value), 0.0)
                for x_value in x_values
            ]
            offset = (
                index - (len(series_values) - 1) / 2
            ) * width
            plt.bar(
                x_indexes + offset,
                y_values,
                width=width,
                label=series_label(
                    series_columns,
                    series_value,
                ),
            )

        plt.legend(fontsize=8)
    else:
        y_values = [
            data.get((x_value,), 0.0)
            for x_value in x_values
        ]
        plt.bar(x_indexes, y_values, width=0.7)

    plt.title(title_for(metric, x_column))
    plt.xlabel(x_column.replace("_", " "))
    plt.ylabel(metric.replace("_", " "))
    plt.xticks(x_indexes, [str(value) for value in x_values])
    plt.grid(True, axis="y", alpha=0.3)
    plt.tight_layout()

    suffix = ""
    if series_columns:
        suffix = "_by_" + "_and_".join(series_columns)

    output_path = (
        output_dir
        / f"bar_{metric}_by_{x_column}{suffix}.png"
    )
    plt.savefig(output_path, dpi=160)
    plt.close()

    return output_path


def save_heatmap(
    rows: list[dict],
    output_dir: Path,
    metric: str,
    x_column: str,
    y_column: str,
) -> Path | None:
    data = aggregate(rows, [x_column, y_column], metric)

    if not data:
        return None

    x_values = unique_values(rows, x_column)
    y_values = unique_values(rows, y_column)

    matrix = np.full(
        (len(y_values), len(x_values)),
        np.nan,
        dtype=float,
    )

    for y_index, y_value in enumerate(y_values):
        for x_index, x_value in enumerate(x_values):
            value = data.get((x_value, y_value))
            if value is not None:
                matrix[y_index, x_index] = value

    if np.isnan(matrix).all():
        return None

    plt.figure(figsize=(8, 5))
    image = plt.imshow(
        matrix,
        aspect="auto",
        origin="lower",
        cmap="viridis",
    )
    plt.colorbar(image, label=metric.replace("_", " "))
    plt.title(
        f"{metric.replace('_', ' ')} heatmap"
    )
    plt.xlabel(x_column.replace("_", " "))
    plt.ylabel(y_column.replace("_", " "))
    plt.xticks(
        np.arange(len(x_values)),
        [str(value) for value in x_values],
    )
    plt.yticks(
        np.arange(len(y_values)),
        [str(value) for value in y_values],
    )
    plt.tight_layout()

    output_path = (
        output_dir
        / (
            f"heatmap_{metric}_by_"
            f"{x_column}_and_{y_column}.png"
        )
    )
    plt.savefig(output_path, dpi=160)
    plt.close()

    return output_path


def infer_csv_path(argument: str) -> Path:
    candidate = Path(argument)

    if candidate.exists():
        return candidate

    return (
        Path("results")
        / "experiments"
        / argument
        / "summary.csv"
    )


def generate_plots(csv_path: Path) -> list[Path]:
    rows = read_rows(csv_path)

    if not rows:
        return []

    output_dir = csv_path.parent / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    metrics = [
        metric
        for metric in METRIC_COLUMNS
        if metric in rows[0] and numeric_column(rows, metric)
    ]

    varying_columns = [
        column
        for column in PARAM_COLUMNS
        if column in rows[0] and len(unique_values(rows, column)) > 1
    ]

    numeric_varying_columns = [
        column
        for column in varying_columns
        if numeric_column(rows, column)
    ]

    categorical_varying_columns = [
        column
        for column in varying_columns
        if not numeric_column(rows, column)
    ]

    saved_paths = []

    for metric in metrics:
        for x_column in numeric_varying_columns:
            series_columns = choose_series_columns(
                varying_columns,
                x_column,
            )
            path = save_line_plot(
                rows,
                output_dir,
                metric,
                x_column,
                series_columns,
            )
            if path:
                saved_paths.append(path)

        for x_column in categorical_varying_columns:
            series_columns = choose_series_columns(
                varying_columns,
                x_column,
            )
            path = save_bar_plot(
                rows,
                output_dir,
                metric,
                x_column,
                series_columns,
            )
            if path:
                saved_paths.append(path)

    if len(numeric_varying_columns) >= 2 and "coverage" in metrics:
        for x_index, x_column in enumerate(numeric_varying_columns):
            for y_column in numeric_varying_columns[x_index + 1:]:
                path = save_heatmap(
                    rows,
                    output_dir,
                    "coverage",
                    x_column,
                    y_column,
                )
                if path:
                    saved_paths.append(path)

    return saved_paths


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Generate comparison plots from a batch experiment summary.csv."
        )
    )
    parser.add_argument(
        "result",
        help=(
            "Batch config name, e.g. planner_comparison_study, "
            "or direct path to summary.csv."
        ),
    )
    args = parser.parse_args()

    csv_path = infer_csv_path(args.result)

    if not csv_path.exists():
        raise FileNotFoundError(
            f"Could not find results CSV: {csv_path}"
        )

    saved_paths = generate_plots(csv_path)

    if not saved_paths:
        print("No plots generated.")
        return

    print("Saved plots:")
    for path in saved_paths:
        print(f"- {path}")


if __name__ == "__main__":
    main()
