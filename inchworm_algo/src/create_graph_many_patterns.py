#!/usr/bin/env python3

from cProfile import label
import csv, sys, pandas as pd
import matplotlib.pyplot as plt
import plotly.graph_objects as go

move_to_days = (16/3600)
WIDTH = 30
HEIGHT = 30
pattern = 0


for j in range(3):
    data = []
    with open(f"../data/{sys.argv[1]}{j}.csv") as csvfile:
        reader = csv.reader(csvfile, delimiter=' ')
        for row in reader:
            # print(row)
            row = row[0].split(',')
            row = [int(x) for x in row]
            row_data = {}
            row_data["inchworm_count"] = row[0]
            row_data["ticks"] = row[1]
            row_data["move_counts"] = row[2: 2+ row_data["inchworm_count"]]
            row_data["state_data"] = {
                    "inchworm_id": [],
                    "Pickup From Depot": [],
                    "Move to Target": [], 
                    "Install Shingle": [],
                    "Move Shingle": [],
                    "Explore": [],
                    "Move to Depot": []
                }
            for i in range(row_data["inchworm_count"]):
                data_index = 2 + row_data["inchworm_count"] + i * 6
                row_sum = 0
                for i in range(6):
                    row_sum += row[data_index + i]
                row_data["state_data"]['inchworm_id'].append(i)
                row_data["state_data"]['Pickup From Depot'].append(100 * row[data_index]/row_sum)
                row_data["state_data"]['Move to Target'].append(100 * row[data_index + 1]/row_sum)
                row_data["state_data"]['Install Shingle'].append(100 * row[data_index + 2]/row_sum)
                row_data["state_data"]['Move Shingle'].append(100 * row[data_index + 3]/row_sum)
                row_data["state_data"]['Explore'].append(100 * row[data_index + 4]/row_sum)
                row_data["state_data"]['Move to Depot'].append(100 * row[data_index + 5]/row_sum)

                # print(row_data["state_data"])
            data.append(row_data)
    inchworms = []
    move_data = []
    average_data = pd.DataFrame()
    for run in data:
        # print(run['state_data'])
        data_frame = pd.DataFrame.from_dict(run['state_data'], orient='index')
        # print(data_frame.drop(['inchworm_id']).mean(axis=1).to_frame())
        # print(average_data)
        move_data.append(max(run['move_counts']) * move_to_days)
        inchworms.append(run['inchworm_count'])
        
        

    print(average_data)
    plt.rcParams['font.size'] = '14'
    plt.scatter(inchworms, move_data, label=f"Pattern {j}")
plt.xlabel("Inchworm Count")
plt.ylabel("Estimated Hours Until Compilation")
plt.title(f"Time to Shingle a {WIDTH}x{HEIGHT} Roof")
plt.legend()

plt.savefig(f"{WIDTH}x{HEIGHT}_{pattern}.png")



plt.savefig(f"{WIDTH}x{HEIGHT}_time.png")
plt.show()