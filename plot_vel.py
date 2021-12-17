import csv
import matplotlib.pyplot as plt
a = []
b = []
c = []
d = []
e = []
f = []
with open('src/foundations_assignment/velocity_plot.csv', newline='') as csvfile:
    spamreader = csv.reader(csvfile, delimiter=' ', quotechar='|')
    for row in spamreader:
        l = row[0].split(",")
        a.append(float(l[0]))
        b.append(float(l[1]))
        c.append(float(l[2]))
        d.append(float(l[3]))
        e.append(float(l[4]))
        f.append(float(l[5]))

temp = plt.plot(c)
plt.plot(f)
plt.show()