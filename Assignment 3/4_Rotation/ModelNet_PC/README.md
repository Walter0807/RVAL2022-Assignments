## Setup

- Compile the CUDA code for PointNet++ backbone. 

```bash
cd pointnet_lib
python setup.py install
```
## Category-level object pose estimation from point clouds

### Dataset

- Download dataset from [ModelNet40](https://lmb.informatik.uni-freiburg.de/resources/datasets/ORION/modelnet40_manually_aligned.tar).

```bash
mkdir dataset && cd dataset
wget https://lmb.informatik.uni-freiburg.de/resources/datasets/ORION/modelnet40_manually_aligned.tar
mkdir modelnet40 && tar xvf modelnet40_manually_aligned.tar -C modelnet40
cd ..
```

- Prepocess data.
```bash
cd code
python prepare.py -d ../dataset/modelnet40 -c airplane
cd ..
```

### Train and Test

- copy and modify `configs/template.config` accordingly. An example can be found in `configs/example.config`.

- train/test

```bash
cd code
python train.py --config example.config
python test.py --config example.config 
cd ..
```

- Use tensorboard to view the logs of training and testing.
```bash
cd exps
tensorboard --logdir .
```

The default loss is L2 loss. We also provide the code of geodesic loss and flow loss. Modify the loss term in `code/train.py` and the other steps are all the same.

