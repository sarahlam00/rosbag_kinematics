import csv
import pandas as pd
filename = "csv_test.csv"  # File name
data = pd.read_csv(filename, encoding='unicode_escape')
# fields = []  # Column names
# rows = []    # Data rows

# with open(filename, 'r') as csvfile:
#     csvreader = csv.reader(csvfile)  # Reader object

#     fields = next(csvreader)  # Read header
#     for row in csvreader:     # Read rows
#         rows.append(row)

#     print("Total no. of rows: %d" % csvreader.line_num)  # Row count

# print('Field names are: ' + ', '.join(fields))

# print('\nFirst 5 rows are:\n')
# for row in rows[:5]:
#     for col in row:
#         print("%10s" % col, end=" ")
#     print('\n')