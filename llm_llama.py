# # bitsandbytes
import torch
from transformers import AutoModelForCausalLM, AutoTokenizer



if torch.backends.mps.is_available():
    torch_dtype = torch.float16
    device = "mps"
    print("Using MPS acceleration")


from transformers import AutoModelForCausalLM, AutoTokenizer

model_id = "mistralai/Mixtral-8x7B-Instruct-v0.1"
tokenizer = AutoTokenizer.from_pretrained(model_id)

model = AutoModelForCausalLM.from_pretrained(model_id, torch_dtype=torch.float16).to(0)

text = "Hello my name is"
inputs = tokenizer(text, return_tensors="pt").to(0)

outputs = model.generate(**inputs, max_new_tokens=20)
print(tokenizer.decode(outputs[0], skip_special_tokens=True))


# text = "Hello my name is"
# inputs = tokenizer(text, return_tensors="pt").to(0)

# outputs = model.generate(**inputs, max_new_tokens=20)
# print(tokenizer.decode(outputs[0], skip_special_tokens=True))
# ## init the model
# self.distil_model = AutoModelForSpeechSeq2Seq.from_pretrained(
#     self.model,
#     torch_dtype=self.torch_dtype,
#     low_cpu_mem_usage=True,
#     use_safetensors=True,
# ).to(self.device)
# # self.distil_model.to(self.device)

# self.processor = AutoProcessor.from_pretrained(self.model)

# self.pipeline = pipeline(
#     "automatic-speech-recognition",
#     model=self.model,
#     tokenizer=self.processor.tokenizer,
#     feature_extractor=self.processor.feature_extractor,
#     max_new_tokens=128,
#     chunk_length_s=15,
#     batch_size=16,
#     torch_dtype=self.torch_dtype,
#     device=self.device,
# )
