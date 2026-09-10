import os, shutil

def delete_all(folder):
    for filename in os.listdir(folder):
        file_path = os.path.join(folder, filename)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            _, ext = os.path.splitext(file_path)
            if ext == ".py" or ext == ".json":
                raise
            else:
                print(f"Failed to delete {file_path}")

EREBUS_FOLDER = ""
try:
    with open("erebus_folder.txt", "r") as f:
        EREBUS_FOLDER = f.read()
except:
    print("Could not read erebus_folder.txt")

if EREBUS_FOLDER == "":
    print("EREBUS_FOLDER is invalid. Won't copy files. Bye!")
elif not os.path.normpath(EREBUS_FOLDER).endswith("game\\controllers\\robot0Controller"):
    print("EREBUS_FOLDER is invalid. It should end in 'game\\controllers\\robot0Controller'. Bye!")
else:        
    print(f"MOVING FILES TO {EREBUS_FOLDER}")
    delete_all(EREBUS_FOLDER)
    shutil.copy("settings.json", os.path.join(EREBUS_FOLDER, "settings.json"))
    shutil.copy("compilado.py", os.path.join(EREBUS_FOLDER, "robot0Controller.py"))
    print("DONE!")
