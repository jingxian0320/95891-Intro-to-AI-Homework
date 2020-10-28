# Import packages
import cartopy.crs as ccrs
import matplotlib.pyplot as plt
import csv


def get_location_coordinates(filepath):
    result = []
    for line in csv.reader(open(filepath, 'r'), delimiter=','):
        if line[0] == "Location":
            continue
        result.append([line[0],float(line[1]),float(line[2])])
    return result

def plot_routine(routine_list):
    locations = get_location_coordinates('location_coordinates.csv')
    plt.figure(figsize=(20,40))
    ax = plt.axes(projection=ccrs.PlateCarree())
    ax.stock_img()
    
    for i in locations:
        city_name = i[0].split(',')[0]
        lat = i[1]
        lon = i[2]
        plt.plot(lon, lat, 'ro')
        plt.text(lon-2, lat, city_name,
                 horizontalalignment='right',
                 transform=ccrs.Geodetic())
    for i in range(len(routine_list)-1):
        plt.plot([locations[routine_list[i]][2], locations[routine_list[i+1]][2]], [locations[routine_list[i]][1], locations[routine_list[i+1]][1]],
                     color='gray', linestyle='--',
                     transform=ccrs.Geodetic())
    
    plt.show()
    
if __name__ == '__main__':
    l = [0, 3, 8, 1, 4, 7, 5, 6, 2, 9, 0]
    plot_routine(l)