#!/usr/bin/env python3

import csv, sys, pandas as pd
import matplotlib.pyplot as plt
import plotly.graph_objects as go

move_to_days = (16/3600)
WIDTH = 30
HEIGHT = 30
pattern = 2

data = []
with open(f"../data/{sys.argv[1]}") as csvfile:
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
    run_average = data_frame.drop(['inchworm_id']).mean(axis=1)
    run_average.name = str(run["inchworm_count"])
    average_data = average_data.append(run_average)
    
    # print(run['move_counts'])
    # fig = go.Figure()
    # for i in range(run['inchworm_count']):
    #     print(data_frame.T.columns.values.tolist()[1:])
    #     print(data_frame.T.iloc[i].values.tolist()[1:])
    #     r_list = data_frame.T.iloc[i].values.tolist()[1:]
    #     r_list.append(data_frame.T.iloc[i].values.tolist()[1])
    #     theta_list = data_frame.T.columns.values.tolist()[1:]
    #     theta_list.append(data_frame.T.columns.values.tolist()[1])
    #     print(r_list)
    #     print(theta_list)
    #     fig.add_trace(go.Scatterpolar(
    #         r=r_list,
    #         theta=theta_list,
    #         fill='none',
    #         name=f'inchworm {i}'
    #     ))
    # fig.update_traces(mode='lines+markers')
    # fig.update_layout(
    #     polar=dict(
    #         radialaxis=dict(
    #         visible=True,
    #         range=[0, data_frame.max() + 20]
    #         )),
    #     showlegend=True
    #     )
    # fig.show()

print(average_data)
plt.rcParams['font.size'] = '14'
plt.scatter(inchworms, move_data)
plt.xlabel("Inchworm Count")
plt.ylabel("Total Estimated Hours Until Compilation")
plt.title(f"Time to Shingle a {WIDTH}x{HEIGHT} Roof with Pattern {pattern}")

plt.savefig(f"{WIDTH}x{HEIGHT}_{pattern}.png")


data_points = [0, 3, 6, 10, 14]
fig = go.Figure()
for i in data_points:
    print(average_data.columns.values.tolist()[1:])
    print(average_data.iloc[i].values.tolist()[1:])
    r_list = average_data.iloc[i].values.tolist()[1:]
    r_list.append(average_data.iloc[i].values.tolist()[1])
    theta_list = average_data.columns.values.tolist()[1:]
    theta_list.append(average_data.columns.values.tolist()[1])
    fig.add_trace(go.Scatterpolar(
        r=r_list,
        theta=theta_list,
        fill='none',
        name=f'Run with {i + 1} inchworms'
    ))
fig.update_traces(mode='lines+markers')
fig.update_layout(
    polar=dict(
        radialaxis=dict(
        visible=True,
        range=[0, data_frame.max()],
        # title = "Percent of Actions Taken"
        )),
    showlegend=True,
    title=f"Share of Actions Taken in Run by Average Inchworm<br><sup>On a {WIDTH}x{HEIGHT} roof with pattern {pattern}</sup>",
    
    font=dict(
        family="DejaVu Sans",
        size=18
    )
    )
fig.write_image(f"{WIDTH}x{HEIGHT}_{pattern}_dist.png", width=1000, height=600)
# plt.savefig(f"{WIDTH}x{HEIGHT}_time01.png")
fig.show()
# plt.show()