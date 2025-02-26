#!/bin/bash

#SBATCH --job-name=eval_overall_mlp_icm_maskppo_training
#SBATCH --output=logs&outputs/eval_overall_mlp_icm_maskppo_output-%j.txt
#SBATCH --partition=mech-cst-mov.cpu.q          # Choose a partition that has GPUs
#SBATCH --time=48:00:00
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=1
#SBATCH --cpus-per-task=1
#SBATCH --mem-per-cpu=16G
#SBATCH --gpus=0 


# Load modules or software if needed
module purge
module load Python/3.11.3-GCCcore-12.3.0
module load MATLAB/2024b
#module load PyTorch/2.1.2-foss-2023a-CUDA-12.1.1
module list

export QT_QPA_PLATFORM=offscreen # Don't open Matlab when running Matlab Engine
export LD_LIBRARY_PATH=$EBROOTMATLAB/bin/glnxa64:$LD_LIBRARY_PATH # Path for Matlab Engine
export MATLABPATH=$MATLABPATH:/mech001.mnt/home/20223748/code_hpc_v2/topology_to_model/ # Path to your matlab code
export MATLABPATH=$MATLABPATH:/mech001.mnt/home/20223748/code_hpc_v2/QSS_Toolbox_Full/
#export PATH="$HOME/.local/bin:$PATH"

# Install Python dependencies
# First, we create and activate a temporary Python venv.
# Then we install the modules that we need into that venv.
#VENVDIR=/scratch-shared/20223748/python-venv-$SLURM_JOB_ID
VENVDIR=python_venv
if [ ! -e $VENVDIR ]; then
	python3 -m venv $VENVDIR
	source $VENVDIR/bin/activate
	pip install --upgrade pip
	pip install \
		numpy~=1.20 \
		typing_extensions==4.11.0 \
		pydantic==1.10.9 \
		stable-baselines3[extra] \
		wandb \
		sb3-contrib \
		matlabengine==24.2.* \
		torch_geometric \
		rllte-core \
		ortools 
else
	echo "Python/pip environment already exists; using existing environment!"
	source $VENVDIR/bin/activate
fi
pip list

# Show commands as they are executed.
#set -x

## python -m pip install tianshou
#pip install --upgrade pip
#pip install numpy~=1.20
#pip install stable-baselines3[extra]
#pip install sb3-contrib
#pip install --upgrade gymnasium
#pip uninstall -y pydantic wandb typing_extensions
#pip install typing_extensions==4.11.0 pydantic==1.10.9 wandb
#pip install matlabengine==24.2.*
## python -m pip install tianshou
#pip install torch_geometric
#pip install rllte-core 


# Login to Weights and Biases
WANDB_API_KEY='04c5a8d2e522a36864ebc81849cd4776668a886d'
wandb login $WANDB_API_KEY

# Define experiment name variable
EXP_NAME="eval_overall_mlp_icm_${SLURM_JOB_ID}"

# Define configuration parameters
POLICY_TYPE="MultiInputPolicy"
ALGORITHM="MASKPPO"                           
TOTAL_TIMESTEPS=3000000
ENV_ID="powertrain_topology_rl/PowertrainEnv_overall-v0"
N_ENVS=1
WANDB_PROJECT="Overall_Env"
WANDB_RUN_NAME="${EXP_NAME}_experiment_${SLURM_JOB_ID}"
MODEL_SAVE_PATH="models/"
FINAL_EPS=0.08
LEARNING_RATE=2e-4
LOCAL_MODEL_SAVE_NAME="${EXP_NAME}"
TRAIN_FREQ=(1, "episode")
STOP_TRAINING_THRESHOLD=1
STOP_TRAINING_RATE=60
STOP_TRAINING_N_EPISODES=50
NUM_SPLIT=1
MAX_COMPONENTS=0
EXTRACT_FEATURES=mlp
CURIOSITY=icm
ACTIVATION=elu
N_STEPS=512
ENT_COEF=0.01

# Create model save directory if it doesn't exist
mkdir -p ${MODEL_SAVE_PATH}

# Execute the training script
python main.py \
    --policy_type ${POLICY_TYPE} \
    --algorithm ${ALGORITHM} \
    --total_timesteps ${TOTAL_TIMESTEPS} \
    --env_id ${ENV_ID} \
    --n_envs ${N_ENVS} \
    --wandb_project ${WANDB_PROJECT} \
    --wandb_run_name ${WANDB_RUN_NAME} \
    --model_save_path ${MODEL_SAVE_PATH} \
    --log_file "logs&outputs/${EXP_NAME}_log-$SLURM_JOB_ID.txt" \
    --test_log_file "logs&outputs/test_${EXP_NAME}_log-$SLURM_JOB_ID.txt" \
    --exploration_final_eps ${FINAL_EPS} \
    --learning_rate ${LEARNING_RATE} \
    --local_model_save_name ${LOCAL_MODEL_SAVE_NAME} \
    --stop_training_threshold ${STOP_TRAINING_THRESHOLD} \
    --stop_training_rate ${STOP_TRAINING_RATE} \
    --stop_training_n_episodes ${STOP_TRAINING_N_EPISODES} \
    --num_split ${NUM_SPLIT} \
    --max_components ${MAX_COMPONENTS} \
    --extract_features ${EXTRACT_FEATURES} \
    --curiosity ${CURIOSITY} \
    --activation ${ACTIVATION} \
    --n_steps ${N_STEPS} \
    --ent_coef ${ENT_COEF} \
