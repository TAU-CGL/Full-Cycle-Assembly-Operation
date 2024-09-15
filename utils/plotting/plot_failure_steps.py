import matplotlib.pyplot as plt
import numpy as np

array = np.load("path/to/failure_steps.npy")

num_bins = 20

bins = np.linspace(0, len(array), num_bins + 1)
bin_indices = np.digitize(np.arange(len(array)), bins) - 1
binned_data = np.array([array[bin_indices == i].sum() for i in range(num_bins)])

bin_labels = [f"{int(bins[i])}-{int(bins[i+1])-1}" for i in range(num_bins)]

plt.bar(bin_labels, binned_data, color="blue", edgecolor="black")
plt.xlabel("Range of Values")
plt.ylabel("Frequency")
plt.title("Steps till Failure Frequency")
plt.xticks(rotation=90)
plt.show()
