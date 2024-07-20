import torch
from transformers import MPNetModel, MPNetTokenizer
from sklearn.metrics.pairwise import cosine_similarity
import pandas as pd
import os

def cosine_sim(embedding1, embedding2):
    return cosine_similarity([embedding1], [embedding2])[0][0]

# Ensure that CUDA is available
device = torch.device("cpu")
if torch.cuda.is_available():
    # Get the current GPU device
    gpu_device = torch.cuda.current_device()
    
    # Get the total and available memory in bytes
    total_memory = torch.cuda.get_device_properties(gpu_device).total_memory
    reserved_memory = torch.cuda.memory_reserved(gpu_device)
    allocated_memory = torch.cuda.memory_allocated(gpu_device)
    free_memory = reserved_memory - allocated_memory

    print(f"Total memory: {total_memory / (1024 ** 3):.2f} GB")
    print(f"Reserved memory: {reserved_memory / (1024 ** 3):.2f} GB")
    print(f"Allocated memory: {allocated_memory / (1024 ** 3):.2f} GB")
    print(f"Free memory: {free_memory / (1024 ** 3):.2f} GB")
    mem_limit = 0.5 * total_memory
    
    # If the free memory is less than the memory limit, use the GPU
    if free_memory > mem_limit:
        device = torch.device("cuda")
else:
    print("CUDA is not available.")
    
print(f"Device: {device}")

# Load the tokenizer and model
tokenizer = MPNetTokenizer.from_pretrained('microsoft/mpnet-base')
model = MPNetModel.from_pretrained('microsoft/mpnet-base')

# Example text
# 
# dataset = pd.read_csv("")

command = "Hello how are you"

# Tokenize the text
inputs = tokenizer(command, return_tensors='pt')

# Move the input data to the GPU
inputs = inputs.to(device)
print(inputs)
# If you need to move embeddings back to the CPU (e.g., for further processing)
# embeddings = inputs.cpu().detach().numpy()
inputs = {key: value.to(device) for key, value in inputs.items()}

# Get the input IDs from the tokenized inputs
input_ids = inputs['input_ids']

# Get the embeddings from the model's embeddings layer
with torch.no_grad():  # We don't need gradients for this operation
    input_embeddings = model.embeddings(input_ids)

# Move the embeddings back to CPU if they are on GPU
input_embeddings = input_embeddings.cpu().detach().numpy()

# Print the input embeddings
print("Input Embeddings:", input_embeddings)

