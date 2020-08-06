from gym.envs.registration import register

# Yumi
register(
    id='yumi-v0',
    entry_point='yumi_gym.envs:YumiEnv',
)