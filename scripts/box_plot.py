import matplotlib.pyplot as plt
import numpy as np

# Data for the box plot
data = {
    'IMG': [332, 164, 412, 276, 310],
    'PCS': [269, 225, 317, 302, 277],
    'PL3D+PCS': [180, 123, 177, 177, 222]
}

# Create a figure and a set of subplots
fig, ax = plt.subplots()

# Draw the horizontal box plot
box = ax.boxplot(data.values(), vert=False, patch_artist=True, labels=data.keys(), showmeans=True)

# Customize colors to match Excel style (blue, orange, gray)
colors = ['#1f77b4', '#ff7f0e', '#7f7f7f']
for patch, color in zip(box['boxes'], colors):
    patch.set_facecolor(color)

# Set the title and labels
ax.set_title('Completion Time (sec) by Method')
ax.set_xlabel('Completion Time (sec)')
ax.set_ylabel('Method')

# Set the x-axis range from 0 to 450
ax.set_xlim(0, 450)

# Adding mean markers
means = [np.mean(values) for values in data.values()]
for i, mean in enumerate(means):
    ax.scatter(mean, i + 1, color='red', marker='x', s=50, zorder=3, label='Mean' if i == 0 else "")

# Plot the actual data points
for i, key in enumerate(data.keys()):
    y = np.random.normal(i + 1, 0.04, size=len(data[key]))  # Adding some jitter for better visibility
    ax.plot(data[key], y, 'ro', alpha=0.5)

# Adding legend for the mean marker
ax.legend()

# Display the plot
plt.show()
