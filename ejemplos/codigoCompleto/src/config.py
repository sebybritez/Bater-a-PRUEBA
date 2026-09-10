import json

class ConfigFile:    
    def __init__(self, filename="settings.json"):
        self.__data = {}
        try: 
            with open(filename, "r") as file:
                self.__data = json.load(file)
        except:
            print(f"Couldn't read config file {filename}. Falling back to default values...")

    def get(self, key, default_value = None):
        return self.__data.get(key, default_value)
    