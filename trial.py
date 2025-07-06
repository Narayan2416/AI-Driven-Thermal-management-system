from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.metrics import mean_squared_error,root_mean_squared_error,mean_absolute_error,r2_score
import pandas as pd
import m2cgen as m2c
import numpy as np

data2 = pd.read_csv('data/prefinal3.csv')
data3 = pd.read_csv('data/prefinal4.csv')
data4 = pd.read_csv('data/prefinal5.csv')
data2.columns = ['atmos_temp', 'flow_rate', 'pwm', 'voltage', 'current', 'battery_temp']
data3.columns = ['atmos_temp', 'flow_rate', 'pwm', 'voltage', 'current', 'battery_temp']
data4.columns = ['atmos_temp', 'flow_rate', 'pwm', 'voltage', 'current', 'battery_temp']
data2['battery_temp'] = data2['battery_temp'].shift(-60)
data3['battery_temp'] = data2['battery_temp'].shift(-60)
data4['battery_temp'] = data2['battery_temp'].shift(-60)
data2 = data2.dropna()
data3 = data2.dropna()
data4 = data2.dropna()
data=pd.concat([data2, data3, data4], axis=0)
data.drop_duplicates(subset=None, keep='first', inplace=True)
X = data.iloc[:,:-1].values  
y = data.iloc[:, -1].values   
poly = PolynomialFeatures(degree=2)
X_poly = poly.fit_transform(X)
x_train, x_test, y_train, y_test = train_test_split(X_poly, y, test_size=0.2, random_state=18)  
model = LinearRegression()
model.fit(x_train, y_train)

ans = model.predict(x_test)
print("Mean Square Error:", mean_squared_error(ans, y_test))
print("Root Mean Square Error:", root_mean_squared_error(ans, y_test))
print("Mean absolute Error:", mean_absolute_error(ans, y_test))
print('\n\n')


#Export the trained model to C code
print("Exporting model to C code...")
model_code = m2c.export_to_c(model)
print(model_code)

print("Model exported successfully.")

