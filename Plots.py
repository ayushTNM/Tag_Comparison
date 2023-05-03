import re
from tqdm import tqdm
from typing import Tuple
import itertools
import matplotlib.pyplot as plt
import matplotlib.backends.backend_pdf as pdf
import numpy as np
import pandas as pd
import textwrap
from pathlib import Path
from multiprocessing import Pool, cpu_count
from shutil import rmtree
from scipy.spatial.transform import Rotation
import warnings

warnings.filterwarnings('ignore', category=pd.io.pytables.PerformanceWarning)

BUILD_LOCATION = "build"

HOLDER_THICKNESS_AT_R0 = 1.2 # cm
HOLDER_THICKNESS_NOT_AT_R0 = 1.3 # cm

TUPLE_COLS = ['est_loc', 'est_pyr', 'real_loc', 'real_pyr']

def adjust_loc(data_df: pd.DataFrame) -> pd.DataFrame:
    """
    Adjusts the real location columns in a DataFrame, 
    based on the thickness of the holder when rotating around the rotation axis.

    Parameters:
        data_df (pd.DataFrame): The DataFrame to adjust.

    Returns:
        pd.DataFrame: The adjusted DataFrame.
    """

    # Copy the real_loc column to a new dataframe
    df = data_df[['real_loc']].copy()

    # Set thickness based on holder radius value
    df['thickness'] = np.where(data_df['holder_r'] == 0, HOLDER_THICKNESS_AT_R0, HOLDER_THICKNESS_NOT_AT_R0)

    # Create a points array for use in adjusting location values
    points = np.column_stack((df['thickness'], np.zeros(len(df)), np.zeros(len(df))))

    # Calculate the difference between the points array and the rotated points
    diffs = abs(points - Rotation.from_euler('z', data_df['holder_p'], degrees=True).apply(points))
    diffs[:, [0, -1]] = diffs[:, [-1, 0]]  # Swap the first and last columns
    diffs[:, 1] = -diffs[:, 1]  # Negate the second column
    diffs[:,0] = -1

    # Add the calculated difference to the real_loc values
    data_df['real_loc'] = list(map(tuple,np.add(np.array((df['real_loc'].to_list())), diffs).tolist()))

    return data_df


# def load_data(results_path: str, storage_name: str) -> pd.DataFrame:
#     """
#     Loads data from CSV files in a directory and returns a concatenated pandas DataFrame.
#     The DataFrame is cached in an HDFStore file, and only files modified since the last
#     call to this function will be loaded.

#     Parameters:
#         results_path (str): Path to the directory containing CSV files.
#         storage_name (str): Name of the HDFStore file containing the cached DataFrame.

#     Returns:
#         pd.DataFrame: Concatenated DataFrame containing data from all CSV files.
#     """

#     storage_path = Path(f"{storage_name}_{results_path.name}.h5")
#     tuple_cols = ['est_loc', 'est_pyr', 'real_loc', 'real_pyr']
#     num_tags = len(list(results_path.glob('*/')))
#     # Open the HDFStore file for writing
#     with pd.HDFStore(storage_path, mode='a', complevel=9) as store:
#         if "/last_modified" in store.keys():
#             last_modified = store.select("/last_modified").to_list()[0]
#         else:
#             last_modified = 0
#         results = sorted(list(results_path.glob('*/*.csv')))
#         results_mtime = max([p.stat().st_mtime for p in results])
#         if len(store.keys()) < num_tags+1 or results_mtime > last_modified:
#             exp_dfs = []
#             progress = tqdm(results)
#             for exp_ind, exp_path in enumerate(progress):
#                 tag_path = exp_path.parent
#                 progress.set_description(f"loading {tag_path.name}")
#                 if f"/{tag_path.name}" in store.keys():
#                     tag_last_modified = store.select(tag_path.name)["mtime"].iloc[0]
#                 else:
#                     tag_last_modified = 0
#                 if (len(store.keys()) <= 2 or (len(store.keys()) < num_tags+1 and f"/{tag_path.name}" not in store.keys()[:-2]) or
#                     tag_path.stat().st_mtime  >= tag_last_modified):

#                     holder_rp = tuple(map(int, re.findall(r'\d+', exp_path.name.replace(tag_path.name, ''))))
#                     exp_df = pd.read_csv(exp_path, converters={col:parse_tuple for col in tuple_cols})
#                     exp_df = exp_df.assign(name=tag_path.name, holder_r=holder_rp[0], holder_p=holder_rp[1], mtime=tag_path.stat().st_mtime)
                    
#                     exp_df = adjust_loc(exp_df)

#                     exp_dfs.append(exp_df)

#                     if exp_ind == len(results)-1 or results[exp_ind+1].parent.name != tag_path.name:
#                         group = pd.concat(exp_dfs)
#                         store.put(tag_path.name, group)
#                         exp_dfs = []
#                         store.put("last_modified",pd.Series([results_mtime]))

#         # Concatenate all DataFrames from the HDFStore file
#         data_df = pd.concat([store.select(name) for name in store.keys()[:-1]])
#         data_df = data_df.set_index(["name","holder_r","holder_p","frame","index"])
#         print(data_df)
#         exit()
#         print("Data loaded.")
#     return data_df


def load_exp(args):
    """
    Load an experiment file and return a pandas DataFrame with the data.

    Parameters:
        args (tuple): A tuple containing the experiment file path, holder position, tag name, and tag modification time.

    Returns:
        pandas.DataFrame: A DataFrame with the experiment data.
    """
    exp_path, holder_rp, tag_path = args

    try:
        exp_df = pd.read_csv(exp_path, converters={col:parse_tuple for col in TUPLE_COLS})
        exp_df = exp_df.assign(name=tag_path.name, holder_r=holder_rp[0], holder_p=holder_rp[1], mtime=tag_path.stat().st_mtime)
        return exp_df
    except Exception as e:
        raise Exception(f"Error processing {exp_path}: {e}")

def load_data(results_path: str, storage_name: str) -> pd.DataFrame:
    """
    Load experiment data from CSV files and store it in an HDF5 file.

    Parameters:
        results_path (str): The path to the directory containing the experiment CSV files.
        storage_name (str): The name to use for the HDF5 file.

    Returns:
        pandas.DataFrame: A DataFrame with the experiment data.
    """
    storage_path = Path(f"{storage_name}_{results_path.name}.h5")
    with pd.HDFStore(storage_path, mode='a', complevel=9) as store:

        # Check if the HDF5 file already contains data
        if "/last_modified" in store.keys():
            last_modified = store.select("/last_modified").to_list()[0]
        else:
            last_modified = 0

        # Get a list of all the experiment CSV files in the directory
        results = sorted(list(results_path.glob('*/*.csv')))

        # Get the modification time of the most recently modified experiment CSV file
        results_mtime = max([p.stat().st_mtime for p in results])

        # Check if any of the experiment CSV files have been modified since the last time the HDF5 file was updated
        if results_mtime > last_modified:

            # Group the experiment CSV files by tag name and holder position
            tag_exp_args = {}
            for exp_path in results:
                tag_path = exp_path.parent
                if tag_path.name not in tag_exp_args:
                    tag_exp_args[tag_path.name] = []
                tag_exp_args[tag_path.name].append((exp_path, 
                                                tuple(map(int, re.findall(r'\d+', exp_path.stem.split('_')[-1]))), # Holer rp
                                                tag_path))
    
            # Load and store the experiment data in parallel using multiple processes
            print("Loading Data")
            with Pool(cpu_count()-1) as p:
                exp_dfs = list(tqdm(p.imap(load_exp, [t for tag in tag_exp_args for t in tag_exp_args[tag]]), total=len(results)))
                store.put("data",pd.concat(exp_dfs))
                store.put("last_modified",pd.Series([results_mtime]))

        # Load the stored experiment data from the HDF5 file
        data_df = pd.concat([store.select(name) for name in store.keys()[:-1]])
        data_df = adjust_loc(data_df)

        # Set the DataFrame index to the tag name, holder position, frame number, and index number
        data_df = data_df.set_index(["name","holder_r","holder_p","frame","index"])

        print("Data loaded.")
    return data_df


def compute_similarity(df:pd.DataFrame) -> None:
    """
    Compute similarity scores between estimated and real locations and rotations, and assign them back to the input
    DataFrame. Assumes the input DataFrame has columns "est_loc", "real_loc", "est_pyr", and "real_pyr".
    
    Parameters:
        df (pd.DataFrame): input pandas DataFrame.
    
    Returns:
        pd.DataFrame: input DataFrame with added columns "similarity", "distance", "angle difference", and "rot similarity".
    """
    
    # Set the weight parameter for location and rotation similarity
    w = 0.5

    # Create a copy of the dataframe with dropped NaN values
    df_dropped = df[TUPLE_COLS].dropna().copy()

    # Compute the Euclidean distance between estimated and real locations
    df_dropped['distance'] = np.linalg.norm(np.vstack(df_dropped["real_loc"]) - np.vstack(df_dropped["est_loc"]), axis=1)

    # Compute the location similarity score using inverse of distance
    loc_similarity = 1 / (1 + df_dropped['distance'])

    # Compute the rotation similarity score using quaternion dot product
    rot_est = Rotation.from_euler('xyz', np.vstack(df_dropped["est_pyr"]), degrees=True)
    rot_real = Rotation.from_euler('xyz', np.vstack(df_dropped["real_pyr"]), degrees=True)
    dot_product = np.sum(rot_est.as_quat() * rot_real.as_quat(), axis=1)
    angle_diff = 2 * np.arccos(np.abs(dot_product))
    df_dropped["angle difference"] = np.rad2deg(angle_diff)
    df_dropped['rot similarity'] = np.cos(angle_diff)

    # Compute the overall similarity score as weighted average of location and rotation similarity
    df_dropped["similarity"] = w * loc_similarity + w * df_dropped['rot similarity']

    # Assign the similarity scores back to the original dataframe
    df_dropped.drop(TUPLE_COLS,axis=1, inplace=True)
    return df.join(df_dropped,how='outer')
    

def plot_data(tag_df: pd.DataFrame, path: Path) -> None:
    """
    Plots and saves visualizations of the tag detection data.

    Parameters:
        tag_df (pd.DataFrame): DataFrame containing the tag detection data.
        path (Path): Path to the directory where all visualizations are to be saved.
    """
    # Extract the occlusion from the path
    occ = Path(path.name)

    # Create a directory to save the figures
    fig_path = path/"Figures"
    fig_path.mkdir(parents=True, exist_ok=True)

    # Create a file to save the plot group
    plotgroup_path = fig_path/f"plotgroup_{occ}.tex"

    # Extract the unique x and y coordinates of the tags
    real_locs = np.vstack(tag_df.groupby(["frame"]).first()["real_loc"])
    x, y = np.unique(real_locs[:, 0]), np.unique(real_locs[:, 2])

    # Define the shape of the plot table and initialize variables
    plot_table_shape = (4,3)
    latex_code = []
    plotgroups = []

    # Create the plot 
    fig, ax = plt.subplots()
    text = fig.text(0.5, -0.02, '', ha="center", fontsize=12)

    # Loop through each tag type
    for (name, name_det_group) in tag_df.groupby('name'):
        # Create a list of all possible combinations of holder rotation, position, and selection
        sel_list = ["first","last"] if name in ["STag", "STag2"] else ["last"]
        det_groups = list(itertools.product(name_det_group.groupby(['holder_r', 'holder_p']),sel_list))

        # Define the path for the PDF containing the plots
        filename = f"plots_{name}.pdf"
        pdf_path = fig_path/filename

        # Create a list of LaTeX code for each plot in the PDF
        latex_page = [f"\\mbox{{\\includegraphics[page={page_nr+1},width={0.99/plot_table_shape[1]} \
                        \\textwidth, height={0.99/plot_table_shape[0]}\\textheight,keepaspectratio]{{{occ/filename}}}}}"
                        for page_nr in range(len(det_groups))]

        # Add the LaTeX page to the list of latex code
        latex_code += latex_page

        # Check if the PDF already exists and has the correct number of pages and modification time
        if pdf_path.exists():
            with open(pdf_path, "rb") as f:
                pagecount = f.read().count(b'/Type /Page')-1
                if pagecount == len(latex_page) and pdf_path.stat().st_mtime >= name_det_group["mtime"].iloc[0]:
                    continue

        # If the PDF does not exist or needs to be updated, create it
        progress = tqdm(det_groups)
        progress.set_description(f"Plotting and saving {name}")
        pdf_path.unlink(missing_ok=True)
        plotgroup_path.unlink(missing_ok=True)
        with pdf.PdfPages(pdf_path) as pdf_pages:
            for ((holder_r,holder_p), det_group),sel_string in progress:
                # Group the detections by frame and select the appropriate detection based on the selection list
                dets = det_group.groupby('frame').agg(sel_string)
                # Reshape the similarity values into a 2D array
                z = np.array(dets["similarity"]).reshape((len(y), len(x)))
                # Create a colormap and set the minimum and maximum values based on the rotation similarity
                cmap = plt.get_cmap('viridis').copy()
                vmin = dets[dets["rot similarity"] > 0]["similarity"].min() if len(dets[dets["rot similarity"] > 0]) > 0 else 0
                vmax = dets[dets["rot similarity"] > 0]["similarity"].max() if len(dets[dets["rot similarity"] > 0]) > 0 else 1
                cmap.set_extremes(under='red')
                # Create the heatmap and add a colorbar
                c = ax.pcolormesh(*np.meshgrid(x, y), z, cmap=cmap,vmin=vmin, vmax = vmax)
                title = f'Similarity {name}, Holder rotation: r{holder_r}p{holder_p}{f", Selection: {sel_string}" if name in ["STag", "STag2"] else ""}'
                ax.set(title=title, xlabel='X (cm)', ylabel='Z (cm)')
                cbar = fig.colorbar(c,extend='min',ticks=np.linspace(vmin, vmax, 8), format='%.3f')
                text.set_text(f'Avg. Similarity: {"{0:.3f}".format(np.round(dets["similarity"].mean(numeric_only=False),decimals=3))}, '
                + f'Rate: {"{0:.3f}".format(np.round(dets["similarity"].notnull().mean(),decimals=3)*100)}')
                # Save the plot to the PDF and remove the colorbar and clear the axis for the next plot
                pdf_pages.savefig(fig, dpi=300, bbox_inches='tight')
                cbar.remove()
                ax.clear()
    # Split the list of plot paths into groups based on the plot table shape
    plotgroups = np.array_split(np.array(latex_code),np.ceil(len(latex_code)/np.prod(plot_table_shape)))

    # Write the plot group to a LaTeX file
    with open(fig_path/f"plotgroup_{occ}.tex","w+") as f, \
        pd.option_context('max_colwidth', None,\
                        'display.latex.escape', False):
        f.write('\\begin{center}\n')
        for plotgroup in plotgroups:
            plotgroup = plotgroup.copy()
            plotgroup.resize(plot_table_shape,refcheck=False)
            group = pd.DataFrame(plotgroup)
            latex_plotgroup = group.style.hide(axis="index").hide(axis="columns").to_latex(column_format="ccc")
            f.write(latex_plotgroup)
        f.write('\\end{center}')

def parse_tuple(x: str) -> Tuple[float]:
    """
    Parses a string representation of a tuple into an actual tuple.

    Parameters:
        x (str): The string representation of the tuple.

    Returns:
        Tuple[float]: The parsed tuple as a tuple of floats.
    """
    try:
        # Attempt to evaluate the string as a tuple
        return eval(x)
    except (ValueError, SyntaxError):
        # If the string cannot be evaluated, return NaN
        return np.nan


def create_table(df, metrics, path):
    """
    Creates LaTeX tables for the given metrics and saves them in the specified path.
    
    Parameters:
        df (pandas.DataFrame): The data to be used for creating the tables.
        metrics (list): A list of metrics to create tables for.
        path (pathlib.Path): The path to save the tables in.
    """
    # Create the directory to save the tables in
    occ = path.name
    table_path = path / "Tables"
    table_path.mkdir(parents=True, exist_ok=True)

    # Loop through each metric
    for metric in metrics:
        # Get the data for STag and STag2
        stags = df[(df.index.get_level_values("name") == "STag") | (df.index.get_level_values("name") == "STag2")][metric].to_frame()

        # Loop through each aggregation method (last for all data, first and last for STag and STag2)
        for ind, (latex_df, aggregation, table_size) in enumerate(zip([df[metric].to_frame(), stags], [['last'], ['first', 'last']], [0.75,0.6])):
            # Skip if the data for all holders is the same as the data for STag and STag2
            if (ind == 0 and np.array_equal(latex_df.index,stags.index)):
                continue

            # Calculate the rate of non-null values for each holder and frame
            rate = latex_df.notnull().groupby(["name", 'holder_r', 'holder_p']).mean() * 100

            # Aggregate the data by holder, roll, pitch, and frame
            latex_df = latex_df.groupby(['name', 'holder_r', 'holder_p', 'frame']).agg(aggregation)

            # Stack the data by holder, roll, pitch, and aggregation method
            latex_df = latex_df.stack(level=1)

            # Aggregate the data by holder, roll, pitch, and aggregation method, and calculate mean, std, and rate if necessary
            latex_df = latex_df.groupby(level=['name', 'holder_r', 'holder_p', -1])[metric].agg(['mean', 'std'])
            if ind == 0 or occ != "No occlusion":
                latex_df = latex_df.assign(rate=rate)
            
            # Set the index names
            latex_df.index.names = [None,"\\shortstack{Holder\\\\roll}", "\\shortstack{Holder\\\\pitch}", None]
            
            # Adjust so that optimal highlighted per tag for STags and across all tags otherwise
            if ind == 1 and occ == "No occlusion":
                latex_df = latex_df.stack().unstack(level=-2)
                # exit()
            else:
                latex_df = latex_df.stack().unstack(level=[0, -2]).round(3)

            # Replace NaN values with "ND" and format the data as strings with 3 decimal places
            latex_df = latex_df.applymap(lambda x: f'{x:.3g}' if not pd.isna(x) else "ND").astype(str)

            # Highlight the maximum or minimum value for each column, depending on the metric
            if len(set(latex_df.columns.get_level_values(0))) > 1:
                table_cols = latex_df.columns.get_level_values(0) != "RUNETag"
                for measure, col in zip(["mean","std","rate"],['blue','magenta','green']):
                    if measure == 'rate':
                        # Only add detection rate if needed
                        if ind == 1 and occ == 'No occlusion':
                            continue
                        # Increase table size if rate present
                        else:
                            table_size+=0.25
                    optimal = max if (metric == 'similarity' and measure == "mean") or measure == 'rate' else min
                    mask = latex_df.index.get_level_values(-1) == measure
                    pref,suf = "{\\color{" + col + "}{\\textbf{", "}}}"
                    # print(latex_df.loc[mask, table_cols])
                    latex_df.loc[mask, table_cols] = latex_df.loc[mask, table_cols].apply(lambda row: 
                                                row.apply(lambda x: f"{pref}{x}{suf}" if x == f'{optimal(row.astype(float)):.3g}' else x),axis=1)
            # fix layout if optimal highlighted per tag for STags
            if ind == 1 and occ == "No occlusion":
                latex_df = latex_df.stack().unstack(level=[0,-1])

            # latex_df = latex_df.applymap(lambda x: f'{x:.3f}' if isinstance(x, float) else x)

            # Unstack the data by holder, roll, pitch, and aggregation method, and sort the columns
            latex_df = latex_df.unstack()
            latex_df = latex_df.sort_index(level=[0, 1], sort_remaining=False, axis=1)

            # Remove the aggregation method column if it is not needed
            if ind == 0:
                latex_df = latex_df.droplevel(-2,axis=1)

            # Generate table format
            table_format = f'{"l"*len(latex_df.index.names)}|{"|".join(textwrap.wrap("r"*len(latex_df.columns), len(set(latex_df.columns.get_level_values(-1)))))}|'

            # Reset the index of the data and remove duplicate values in the first
            latex_df = latex_df.reset_index(col_level=-1)
            latex_df.iloc[:, 0] = latex_df.iloc[:, 0].where(~latex_df.iloc[:, 0].duplicated(), '')
            
            # Write the LaTeX code for the table to a file
            with open(f'{table_path/metric }{"_stags" if ind == 1 else ""}.tex', 'w+') as f, pd.option_context('max_colwidth', None,
                                                                    'display.latex.escape', False,
                                                                    'display.float_format', '{:.3g}'.format):
                # Format the data as a LaTeX table and add formatting for horizontal lines and column widths
                latex_str = latex_df.style.hide(axis="index").set_table_styles([
                            {'selector': 'midrule', 'props': ':hline;'},
                            {'selector': 'bottomrule', 'props': ':hline;'}
                            ], overwrite=False).to_latex(column_format=table_format, multicol_align="c|")
                latex_str = f"\\begin{{table}}[ht]\n\\centering\n\\def\\arraystretch{{1.5}}\n\\resizebox{{{table_size}\\textwidth}}{{!}}{{{latex_str[:-1]}}}\n\\end{{table}}"
                latex_str = re.sub(r"(?<=\\\\\n)([346])", r"\\hline\n\1", latex_str)
                f.write(latex_str)
                
    
if __name__ == "__main__":
    # start = time.time()
    list_cols = ['est_loc', 'est_pyr', 'real_loc', 'real_pyr']

    # test = pd.read_csv(resultsPath/"STag2"/"STag2_r60p75.csv",converters={k: parse_tuple for k in list_cols})
    # print(test)
    # print(time.time()-start)

    # for expPath in resultsPath.glob("*/*.json"):
    #     # print(expPath)
    #     with open(expPath, "r") as f:
    #         expList = json.load(f)
    #     for indi,i in enumerate(expList):
    #         for indj,j in  enumerate(i):
    #             for key in j.keys():
    #                 if isinstance(j[key],list):
    #                     # print("yeaa")
    #                     j[key] = f"{tuple(j[key])}"
    #     df = pd.json_normalize(expList).stack().apply(pd.Series)
    #     df=df.reindex(columns=["id","est_loc","real_loc","est_pyr","real_pyr","det_time_ns"])
    #     df.index.names=["frame","index"]
    #     df["id"] = df["id"].astype(object)
    #     df["id"] = pd.to_numeric(df["id"],errors="coerce")
    #     df=df.reset_index()
    #     df["frame"]+=1
    #     print(df)
    #     test = ""
    #     print(df)
    #     # df.to_csv(test)
    #     splitted = df.to_csv(index=False).split("\n")
    #     for lineInd in range(len(splitted)):
    #         splitted[lineInd] = splitted[lineInd].replace(", ",",")
    #         if "," in splitted[lineInd]:
    #             test = splitted[lineInd].split(",")
    #             test[2] = test[2].replace(".0","")
    #             test[-1] = test[-1].replace(".0","")
    #             splitted[lineInd] = ",".join(test)

    # #     # splitted[1].replace(".0","")
    # #     # splitted[-1].replace(".0","")
    # #     print("\n".join(splitted))
    #     with open(f"{expPath.parent/expPath.stem}.csv","w") as o:
    #         o.write("\n".join(splitted))

    storedDataName = "stored_data"
    for occ in ["No occlusion", "TopRight occlusion", "Half occlusion"]:
    # for occ in ["Half occlusion"]:
        print("Processing",occ)
        resultsPath = Path(BUILD_LOCATION)/"results"/occ
        if resultsPath.exists() and resultsPath.stat().st_size != 0:
            data = load_data(resultsPath,storedDataName)
            # Select only detections with ID 8 for the RUNE-Tag and ID 0 for all other tags
            correct_det_group = data.loc[((data['id'] == 8) & (data.index.get_level_values("name") == "RUNETag")) |
                                        ((data['id'] == 0) & (data.index.get_level_values("name") != "RUNETag"))
                                        ].copy().reindex(data.index).assign(real_loc = data["real_loc"], real_pyr = data["real_pyr"], mtime = data["mtime"])

            missclassifications = pd.concat([data,correct_det_group]).drop_duplicates(keep=False)

            # Compute similarity scores
            correct_det_group = compute_similarity(correct_det_group)
            print(correct_det_group)
            # exit()
            # print(np.array(correct_det_group.index.get_level_values("frame")), np.array(np.sort(missclassifications[missclassifications.index.get_level_values('name') == "STag2"].index.get_level_values('frame'))))
            # print(np.isin(np.array(correct_det_group.index.get_level_values("frame")), np.array(missclassifications.index.get_level_values('frame'))))
            # with pd.option_context('max_colwidth', None,'display.max_rows', None, 'display.max_columns', None):
            #     print(correct_det_group[(correct_det_group["angle difference"] > 90) & (correct_det_group["similarity"] > 0) &
            #             ((correct_det_group.index.get_level_values("name") == 'STag2'))][["id","angle difference","similarity"]].loc[missclassifications.index]
            #     )
            # exit()
            path = Path("Vizualisations")/occ
            create_table(correct_det_group,["similarity","distance","angle difference"],path)

            plot_data(correct_det_group, path)