![MilliNoise Logo](/images/millinoise-logo.png)

# MilliNoise: a millimeter-wave sparse point cloud dataset in indoor scenarios

Millimeter-wave radar sensors sensors produce PCs that are much sparser and noisier than other PC data (e.g., LiDAR), yet they are more robust in challenging conditions such as in the presence of fog, dust, smoke, or rain. This paper presents MilliNoise, a millimeter-wave sparse point cloud dataset captured in indoor scenarios. Each point in MilliNoise is accurately labelled as true/noise point by leveraging known information of the scenes and a motion capture system to obtain the ground truth position of the moving robot. Each frame is carefully pre-processed to produce a fixed number of points for each cloud, enabling the employment of classification tools which require such characteristic. MilliNoise has been post-processed as well to allow moving the denoising task into the regression framework, by labelling each point with the distance to its closest obstacle in the scene.

![MilliNoise acquisition system](/images/millinoise-da.jpg)

## Scenes Collected

| Scene 0 | Scene 1 | Scene 5 |
| --- | --- | --- |
| <img src="./images/run_4.gif" width="100%"/> | <img src="./images/run_3.gif" width="100%"/> | <img src="./images/run_51.gif" width="100%"/> |

| Scene 6 | Scene 8 | Scene 9 |
| --- | --- | --- |
| <img src="./images/run_61.gif" width="100%"/> | <img src="./images/run_71.gif" width="100%"/> | <img src="./images/run_81.gif" width="100%"/> |


# Citing MilliNoise
If you find MilliNoise useful or use MilliNoise in your research, please cite it in your publications.

```
@inproceedings{10.1145/3625468.3652189,
    author = {Brescia, Walter and Gomes, Pedro and Toni, Laura and Mascolo, Saverio and De Cicco, Luca},
    title = {MilliNoise: a Millimeter-wave Radar Sparse Point Cloud Dataset in Indoor Scenarios},
    year = {2024},
    publisher = {Association for Computing Machinery},
    url = {https://doi.org/10.1145/3625468.3652189},
    doi = {10.1145/3625468.3652189},
    abstract = {Millimeter-wave (mmWave) radar sensors produce Point Clouds (PCs) that are much sparser and noisier than other PC data (e.g., LiDAR), yet they are more robust in challenging conditions such as in the presence of fog, dust, smoke, or rain. This paper presents MilliNoise, a point cloud dataset captured in indoor scenarios through a mmWave radar sensor installed on a wheeled mobile robot. Each of the ~12M points in the MilliNoise dataset is accurately labeled as true/noise point by leveraging known information of the scenes and a motion capture system to obtain the ground truth position of the moving robot. Each frame is carefully pre-processed to produce a fixed number of points for each cloud, enabling the classification tools which require data with a fixed shape. Moreover, MilliNoise has been post-processed by labeling each point with the distance to its closest obstacle in the scene which allows casting the denoising task into the regression framework. Along with the dataset, we provide researchers with the tools to visualize the data and prepare it for statistical and machine learning analysis. MilliNoise is available at: https://github.com/c3lab/MilliNoise},
    booktitle = {Proceedings of the 15th ACM Multimedia Systems Conference},
    keywords = {mmWave radar, sparse point cloud, denoising, dataset},
    location = {Bari, Italy},
    series = {MMSys '24}
}
```