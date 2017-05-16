source ~/tensorflow/bin/activate
export CUDA_HOME=/usr/local/cuda-8.0
export LD_LIBRARY_PATH="/usr/local/cuda-8.0/lib64/":$LD_LIBRARY_PATH
export LD_LIBRARY_PATH="/usr/local/cudnn-5.1/lib64":$LD_LIBRARY_PATH
python mainControlTF.py
