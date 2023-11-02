import gymnasium as gym

from stable_baselines3 import PPO

# Create the environment
env = gym.make('LunarLander-v2', render_mode="rgb_array")  # continuous: LunarLanderContinuous-v2

# Instantiate the agent
model = PPO('MlpPolicy', env, verbose=1)
# Train the agent and display a progress bar
model.learn(total_timesteps=100000, progress_bar=True)

# Enjoy trained agent
vec_env = model.get_env()
obs = vec_env.reset()
for i in range(10000):
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, dones, info = vec_env.step(action)
    vec_env.render("human")