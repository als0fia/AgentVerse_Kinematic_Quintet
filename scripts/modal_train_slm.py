"""Modal Cloud Training Script for Fine-Tuning the Swarm ReAct SLM.

Trains a lightweight model (Qwen2.5-0.5B-Instruct or Qwen2.5-1.5B-Instruct) on our
generated ReAct incident triage dataset (data/orchestrator_slm_train.jsonl) using
LoRA / PEFT on an Nvidia GPU in Modal. Saves trained adapter weights to a Modal Volume.
"""
from pathlib import Path
import modal

ROOT = Path(__file__).resolve().parents[1]

app = modal.App("antenna-swarm-slm-training")

# Persistent storage volume for model weights and checkpoints
weights_volume = modal.Volume.from_name("antenna-slm-weights", create_if_missing=True)

# Training container image with CUDA PyTorch and Hugging Face training stack
train_image = (
    modal.Image.debian_slim(python_version="3.11")
    .pip_install(
        "torch>=2.2.0",
        "transformers>=4.40.0",
        "peft>=0.10.0",
        "trl>=0.8.0",
        "datasets>=2.18.0",
        "accelerate>=0.28.0",
    )
    .add_local_dir(str(ROOT / "data"), remote_path="/root/data")
)


@app.function(
    image=train_image,
    gpu="T4",  # Budget-friendly GPU (~$0.59/hr, trains in ~5 minutes for ~$0.05)
    timeout=1800,
    volumes={"/root/weights": weights_volume},
)
def train_slm(
    model_id: str = "Qwen/Qwen2.5-0.5B-Instruct",
    num_epochs: int = 3,
    batch_size: int = 2,
    learning_rate: float = 2e-4,
) -> dict:
    """Fine-tunes the Swarm ReAct SLM on Modal GPU using LoRA and SFTTrainer."""
    import os
    import torch
    from datasets import load_dataset
    from transformers import AutoModelForCausalLM, AutoTokenizer, TrainingArguments
    from peft import LoraConfig, get_peft_model
    from trl import SFTTrainer

    print(f"\n[1/5] Loading dataset from /root/data/...")
    dataset = load_dataset(
        "json",
        data_files={
            "train": "/root/data/orchestrator_slm_train.jsonl",
            "eval": "/root/data/orchestrator_slm_eval.jsonl",
        },
    )
    print(f"      Train episodes: {len(dataset['train'])}, Eval episodes: {len(dataset['eval'])}")

    print(f"\n[2/5] Loading base tokenizer & model: {model_id}...")
    tokenizer = AutoTokenizer.from_pretrained(model_id, trust_remote_code=True)
    if tokenizer.pad_token is None:
        tokenizer.pad_token = tokenizer.eos_token

    model = AutoModelForCausalLM.from_pretrained(
        model_id,
        torch_dtype=torch.float16,
        device_map="auto",
        trust_remote_code=True,
    )

    print(f"\n[3/5] Configuring LoRA adapter (r=16, alpha=32)...")
    peft_config = LoraConfig(
        r=16,
        lora_alpha=32,
        target_modules=["q_proj", "k_proj", "v_proj", "o_proj", "gate_proj", "up_proj", "down_proj"],
        lora_dropout=0.05,
        bias="none",
        task_type="CAUSAL_LM",
    )
    model = get_peft_model(model, peft_config)
    model.print_trainable_parameters()

    output_dir = "/tmp/slm_checkpoint"
    final_output_dir = "/root/weights/antenna_qwen_react_adapter"
    os.makedirs(final_output_dir, exist_ok=True)

    # Format chat messages using the tokenizer's chat template
    def formatting_prompts_func(example):
        msgs = example["messages"]
        if isinstance(msgs, list) and len(msgs) > 0 and isinstance(msgs[0], list):
            # Batched mode
            return [tokenizer.apply_chat_template(m, tokenize=False, add_generation_prompt=False) for m in msgs]
        else:
            # Single example mode
            return tokenizer.apply_chat_template(msgs, tokenize=False, add_generation_prompt=False)

    # Modern TRL uses SFTConfig (inherits from TrainingArguments)
    from trl import SFTConfig, SFTTrainer

    sft_config = SFTConfig(
        output_dir=output_dir,
        num_train_epochs=num_epochs,
        per_device_train_batch_size=batch_size,
        gradient_accumulation_steps=4,
        learning_rate=learning_rate,
        lr_scheduler_type="cosine",
        warmup_steps=10,
        fp16=torch.cuda.is_available(),
        logging_steps=5,
        save_strategy="no",
        eval_strategy="epoch",
        max_length=1536,
        report_to="none",
    )

    print(f"\n[4/5] Starting SFT training loop on GPU...")
    trainer = SFTTrainer(
        model=model,
        train_dataset=dataset["train"],
        eval_dataset=dataset["eval"],
        formatting_func=formatting_prompts_func,
        args=sft_config,
    )
    train_result = trainer.train()

    print(f"\n[5/5] Saving fine-tuned adapter to persistent Modal Volume...")
    trainer.model.save_pretrained(final_output_dir)
    tokenizer.save_pretrained(final_output_dir)
    weights_volume.commit()

    print(f"      Successfully saved adapter weights to {final_output_dir}!")
    return {
        "status": "COMPLETED",
        "train_loss": train_result.training_loss,
        "epochs": num_epochs,
        "adapter_path": final_output_dir,
    }


@app.local_entrypoint()
def main():
    print("=" * 70)
    print(" LAUNCHING SWARM SLM FINE-TUNING ON MODAL GPU")
    print("=" * 70)
    print("-> Base model: Qwen/Qwen2.5-0.5B-Instruct")
    print("-> Dataset: 150 train episodes + 30 eval episodes (ReAct traces)")
    print("-> Hardware: Nvidia T4 GPU (~5-8 mins runtime)\n")

    result = train_slm.remote()
    print("\n" + "=" * 70)
    print(" TRAINING SUMMARY:")
    print(f" Status: {result['status']}")
    print(f" Final Loss: {result['train_loss']:.4f}")
    print(f" Adapter saved at: {result['adapter_path']} (in volume 'antenna-slm-weights')")
    print("=" * 70)
