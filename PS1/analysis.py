import pandas as pd
df = pd.read_csv("results.csv")
print("Raw data:")
print(df.head())
summary = (
    df.groupby("heuristic")
    .agg(
        average_nodes_expanded=(
            "nodes_expanded",
            "mean"
        ),
        average_execution_time=(
            "execution_time",
            "mean"
        ),
        average_path_length=(
            "path_length",
            "mean"
        ),
        success_rate=(
            "success",
            "mean"
        )
    )
    .reset_index()
)
summary["success_rate"] *= 100
summary[
    "average_nodes_expanded"
] = summary[
    "average_nodes_expanded"
].round(2)
summary[
    "average_execution_time"
] = summary[
    "average_execution_time"
].round(6)
summary[
    "average_path_length"
] = summary[
    "average_path_length"
].round(2)
summary[
    "success_rate"
] = summary[
    "success_rate"
].round(2)
print("\nPerformance Summary:")
print(
    summary.to_string(index=False)
)
summary.to_csv(
    "summary.csv",
    index=False
)
print("\nSummary saved to summary.csv")