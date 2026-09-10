from config import ConfigFile
from director import Director
import cProfile

config = ConfigFile()

def main():
    director = Director(config)
    director.start()

# HACK(Richo): Enable this branch to run the profiler
if config.get("profile_enabled", False):
    cProfile.run('main()', "stats.prof")
else:
    main()