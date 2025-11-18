import pandas as pd
import numpy as np

# Read the CSV file
df = pd.read_csv('y_error_ext75.csv')

# Calculate absolute values
abs_errors = df.abs()

# Calculate quartiles (Q1, Q2, Q3, Q4)
Q1 = abs_errors.quantile(0.25)
Q2 = abs_errors.quantile(0.50)  # Median
Q3 = abs_errors.quantile(0.75)
Q4 = abs_errors.max()  # Maximum value

# Calculate RMSE
rmse = np.sqrt((df ** 2).mean())

# Print results
print("Quartiles of Absolute Errors:")
print(f"Q1 (25th percentile): \n{Q1}\n")
print(f"Q2 (50th percentile/Median): \n{Q2}\n")
print(f"Q3 (75th percentile): \n{Q3}\n")
print(f"Q4 (Maximum): \n{Q4}\n")

print("Root Mean Square Error (RMSE):")
print(rmse)