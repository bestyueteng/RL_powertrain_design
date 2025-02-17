#!/bin/bash

#SBATCH --job-name=cp_generation_3444
#SBATCH --output=cp_generation_3444.txt
#SBATCH --partition=tue.default.q           # Choose a partition that has GPUs
#SBATCH --time=120:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem-per-cpu=16G
#SBATCH --gpus=0

# Load modules or software if needed
module purge
module load Python
module load MATLAB
module load PyTorch/2.1.2-foss-2023a-CUDA-12.1.1

export QT_QPA_PLATFORM=offscreen # Don't open Matlab when running Matlab Engine
export LD_LIBRARY_PATH=$EBROOTMATLAB/bin/glnxa64:$LD_LIBRARY_PATH # Path for Matlab Engine
export MATLABPATH=$MATLABPATH:/mech001.mnt/home/20223748/code_hpc/topology_to_model/ 
export MATLABPATH=$MATLABPATH:/mech001.mnt/home/20223748/code_hpc/QSS_Toolbox_Full/
export PATH="$HOME/.local/bin:$PATH"

chmod +x /mech001.mnt/home/20223748/

# Install Python dependencies
python -m pip install stable-baselines3[extra]
pip install sb3-contrib
pip uninstall -y pydantic wandb typing_extensions
pip install typing_extensions==4.11.0 pydantic==1.10.9 wandb
python -m pip install matlabengine
python -m pip install tianshou
python -m pip install torch_geometric

# Define experiment name variable

EXP_NAME="random_ppo_mlp"

# Define configuration parameters
ALGORITHM="PPO"     

python cp_generation.py 