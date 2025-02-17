# Overview

This repository implements a deep reinforcement learning framework for automated powertrain design in battery electric vehicles. The code integrates RL techniques with an extensive component library and functional constraints to efficiently explore and optimize complex system-level topologies, significantly reducing design time while achieving cost-effective solutions.

---

## Project Environment Setup

This project requires:
- **Python 3.11** (specifically, Python/3.11.3-GCCcore-12.3.0 on HPC)
- **MATLAB 2024b** with MATLAB Engine support
- A variety of Python packages including, but not limited to, `numpy`, `pydantic`, `stable-baselines3`, and `matlabengine`.

The repository supports two deployment scenarios:
1. **HPC/Cluster Environment:** Uses environment modules to load the required software.
2. **Local Environment:** Uses a virtual environment on your desktop or laptop without a module system.

---

## HPC/Cluster Environment Setup

When running on an HPC system, the environment is managed using modules. Follow these steps to set up your HPC environment.
It can be simpy run with the command:
```bash
sbatch run.sh
```
Or can be manually config using the instructions below:

### Prerequisites

- **Module System:** Ensure your HPC system supports the `module` command.
- **Required Modules:**
  - `Python/3.11.3-GCCcore-12.3.0`
  - `MATLAB/2024b`
- **Filesystem Access:** You must have write permission to create directories for Python virtual environments.

### Setup Instructions

1. **Load Required Modules**

   Purge existing modules to avoid conflicts, then load the necessary modules:

   ```bash
   module purge
   module load Python/3.11.3-GCCcore-12.3.0
   module load MATLAB/2024b
   module list
   ```

2. **Configure MATLAB Environment Variables**

  Set environment variables to run MATLAB Engine in offscreen mode and ensure MATLAB libraries and paths are correctly set:
  
  ```bash
  export QT_QPA_PLATFORM=offscreen  # Prevent MATLAB GUI from opening
  export LD_LIBRARY_PATH=$EBROOTMATLAB/bin/glnxa64:$LD_LIBRARY_PATH  # MATLAB Engine library path
  export MATLABPATH=$MATLABPATH:<your_repository>/topology_to_model/
  export MATLABPATH=$MATLABPATH:<your_repository>/QSS_Toolbox_Full/
  ```

3. **Set up Python Environment**
  ```bash
  VENVDIR=<your_env_name>
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
          ortools 
  else
      echo "Python/pip environment already exists; using existing environment!"
      source $VENVDIR/bin/activate
  fi
  pip list
  ```

4. **Run the script**

```
sbatch run.sh
```

---

## Local Environment Setup

**Required Modules:**
  - `Python/3.11.3-GCCcore-12.3.0`
  - `MATLAB/2024b`

1. **Clone the repository**:

```
git clone https://github.com/bestyueteng/RL_powertrain_design.git
cd RL_powertrain_design
```

2. **Install requirements**
For python dependencies:
```
pip install -r requirements.txt
```
For Matlab, should add ``QSS_Toolbox_Full`` and ``topology_to_model`` repositories in your environmental path.

3. Run the main.py with configs

```
python main.py
```
