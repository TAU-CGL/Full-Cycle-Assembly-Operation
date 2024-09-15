import pickle
import pandas as pd

# path_name = './tmp/abc_light_inflated_path'
path_name = './tmp/06397_inflated_path'

# convert a 6D path (xyz + unit quaternion) into a csv format
if __name__ == "__main__":
    with open(path_name+'.pkl', 'rb') as fp:
        solution = pickle.load(fp)
        df = pd.DataFrame(solution)
        df.to_csv(path_name+'.csv')
 