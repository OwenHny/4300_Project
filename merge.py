import pandas as pd

df1 = pd.read_csv('IP2LOCATION-LITE-ASN.csv')
df2 = pd.read_csv('IP2LOCATION-LITE-DB5.csv')

merged_df = pd.merge(df1, df2, on=['0', '16777215'], how='inner')

print(merged_df.columns)

cols = ["0" ,"16777215", "-_x", "0.000000", "0.000000.1"]

merged_df[cols].to_csv('out.csv', index=False)