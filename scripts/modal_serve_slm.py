"""Modal Serverless Inference Service for Fine-Tuned Swarm ReAct SLM (`scripts/modal_serve_slm.py`).

Loads base model `Qwen/Qwen2.5-0.5B-Instruct` with the fine-tuned LoRA adapter
from persistent Modal Volume `antenna-slm-weights` and serves low-latency ReAct
deliberative reasoning for swarm robotics rescue incidents.
"""
from __future__ import annotations

import os
from pathlib import Path
import modal

app = modal.App("antenna-swarm-slm-service")

# Mount existing weights volume
weights_volume = modal.Volume.from_name("antenna-slm-weights")

# Inference image with PyTorch and Hugging Face PEFT
infer_image = (
    modal.Image.debian_slim(python_version="3.11")
    .pip_install(
        "torch>=2.2.0",
        "transformers>=4.40.0",
        "peft>=0.10.0",
        "accelerate>=0.28.0",
    )
)


@app.cls(
    image=infer_image,
    gpu="T4",
    timeout=600,
    volumes={"/root/weights": weights_volume},
)
class SwarmOrchestratorModel:
    @modal.enter()
    def load_model(self):
        import torch
        from transformers import AutoModelForCausalLM, AutoTokenizer
        from peft import PeftModel

        base_model_id = "Qwen/Qwen2.5-0.5B-Instruct"
        adapter_path = "/root/weights/antenna_qwen_react_adapter"

        print(f"[Modal Worker] Loading base tokenizer & model: {base_model_id}...")
        self.tokenizer = AutoTokenizer.from_pretrained(base_model_id, trust_remote_code=True)
        if self.tokenizer.pad_token is None:
            self.tokenizer.pad_token = self.tokenizer.eos_token

        base_model = AutoModelForCausalLM.from_pretrained(
            base_model_id,
            torch_dtype=torch.float16,
            device_map="auto",
            trust_remote_code=True,
        )

        print(f"[Modal Worker] Attaching fine-tuned adapter from {adapter_path}...")
        self.model = PeftModel.from_pretrained(base_model, adapter_path)
        self.model.eval()
        print("[Modal Worker] Model & Adapter successfully loaded in GPU VRAM!")

    @modal.method()
    def infer(self, prompt: str, max_new_tokens: int = 512, temperature: float = 0.1) -> str:
        """Runs autoregressive ReAct generation for swarm incident telemetry."""
        import torch

        inputs = self.tokenizer(prompt, return_tensors="pt").to(self.model.device)
        with torch.no_grad():
            output_tokens = self.model.generate(
                **inputs,
                max_new_tokens=max_new_tokens,
                temperature=temperature if temperature > 0 else None,
                do_sample=temperature > 0,
                pad_token_id=self.tokenizer.pad_token_id,
                eos_token_id=self.tokenizer.eos_token_id,
            )

        # Slice off input tokens to return only newly generated ReAct tokens
        generated = output_tokens[0][inputs.input_ids.shape[1] :]
        return self.tokenizer.decode(generated, skip_special_tokens=True).strip()


@app.local_entrypoint()
def test_entrypoint():
    """Smoke test to verify remote model response."""
    service = SwarmOrchestratorModel()
    sample_prompt = (
        "<|im_start|>system\n"
        "You are the System-2 Deliberative Swarm Orchestrator for agricultural robots in a rice terrace.\n"
        "<|im_end|>\n"
        "<|im_start|>user\n"
        "NEW INCIDENT TELEMETRY DETECTED:\n"
        "{\n"
        '  "incident_id": "inc-smoke-test-001",\n'
        '  "time_s": 45.2,\n'
        '  "stranded_robot_id": "ant-alpha",\n'
        '  "failure_mode": "contagious_mud_trap",\n'
        '  "slip_ratio": 0.88,\n'
        '  "local_moisture": 0.68,\n'
        '  "local_firmness": 0.36,\n'
        '  "available_helpers": [{"robot_id": "scout", "seeds_remaining": 0, "battery_ratio": 0.85}]\n'
        "}\n\n"
        "Analyze this incident step-by-step using ReAct. Output your final mission decision.\n"
        "<|im_end|>\n"
        "<|im_start|>assistant\n"
    )
    print("Testing remote inference on fine-tuned Modal model...")
    response = service.infer.remote(sample_prompt)
    print("\n--- MODEL RESPONSE ---")
    print(response)
    print("----------------------")
