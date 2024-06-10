import matplotlib.pyplot as plt
import pandas as pd
from sklearn.linear_model import LinearRegression
file_path = './stompc/experiments/training_time.csv'
data = pd.read_csv(file_path)
# Extract the relevant columns
X = data[['total_cells']].values
y = data['training_time'].values

# Fit a linear regression model
model = LinearRegression()
model.fit(X, y)

# Predict values for the best fitting line
y_pred = model.predict(X)

# Plot the data points
plt.scatter(X, y, color='blue', label='Data points')

# Plot the best fitting line
plt.plot(X, y_pred, color='red', linewidth=2, label='Best fitting line')

# Add labels and title
plt.xlabel('Number of Cells')
plt.ylabel('Learning Time (Seconds)')
plt.title('Learning Time vs. Number of Cells')
plt.legend()

# Show the plot
plt.show()