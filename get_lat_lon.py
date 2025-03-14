import requests


def get_lat_lon(postcode):
    url = f"https://api.postcodes.io/postcodes/{postcode}"
    response = requests.get(url)

    if response.status_code == 200:
        data = response.json()
        lat = data['result']['latitude']
        lon = data['result']['longitude']
        return lat, lon
    else:
        print("Invalid postcode or API error")
        return None
