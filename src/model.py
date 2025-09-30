import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import layers, models, optimizers

class PPOAgent:
    def __init__(self, state_dim, action_dim,
                 actor_lr=3e-4, critic_lr=1e-3,
                 gamma=0.99, lam=0.95,
                 clip_ratio=0.2, update_epochs=10,
                 batch_size=64):
        self.state_dim = state_dim
        self.action_dim = action_dim
        self.gamma = gamma
        self.lam = lam
        self.clip_ratio = clip_ratio
        self.update_epochs = update_epochs
        self.batch_size = batch_size
        self._build_models(actor_lr, critic_lr)
        self._init_buffers()

    def _build_models(self, actor_lr, critic_lr):
        # Actor
        inp = layers.Input(shape=self.state_dim)
        x = layers.Flatten()(inp)
        x = layers.Dense(128, activation='relu')(x)
        x = layers.Dense(128, activation='relu')(x)
        self.logits_layer = layers.Dense(self.action_dim)
        out = self.logits_layer(x)
        self.actor = models.Model(inp, out)
        self.actor.compile(optimizer=optimizers.Adam(actor_lr))

        # Critic
        inp_v = layers.Input(shape=self.state_dim)
        y = layers.Flatten()(inp_v)
        y = layers.Dense(128, activation='relu')(y)
        y = layers.Dense(128, activation='relu')(y)
        val = layers.Dense(1)(y)
        self.critic = models.Model(inp_v, val)
        self.critic.compile(optimizer=optimizers.Adam(critic_lr), loss='mse')

    def _init_buffers(self):
        self.obs_buf = []
        self.act_buf = []
        self.rew_buf = []
        self.val_buf = []
        self.logp_buf = []
        self.done_buf = []

    def get_action(self, state):
        logits = self.actor.predict(state[None], verbose=0)
        probs = tf.nn.softmax(logits).numpy().flatten()
        action = np.random.choice(self.action_dim, p=probs)
        logp = np.log(probs[action] + 1e-8)
        value = self.critic.predict(state[None], verbose=0)[0,0]
        return action, logp, value

    def store_transition(self, obs, act, logp, val, rew, done):
        self.obs_buf.append(obs)
        self.act_buf.append(act)
        self.logp_buf.append(logp)
        self.val_buf.append(val)
        self.rew_buf.append(rew)
        self.done_buf.append(done)

    def finish_episode(self):
        # compute advantages and returns
        rets, advs = self._compute_gae()
        # normalize adv
        advs = (advs - np.mean(advs)) / (np.std(advs) + 1e-8)
        # prepare dataset
        dataset = tf.data.Dataset.from_tensor_slices((
            np.array(self.obs_buf),
            np.array(self.act_buf),
            np.array(self.logp_buf),
            rets,
            advs
        )).shuffle(len(self.obs_buf)).batch(self.batch_size)
        # update
        for _ in range(self.update_epochs):
            for obs_batch, act_batch, logp_old, ret_batch, adv_batch in dataset:
                with tf.GradientTape() as tape:
                    logits = self.actor(obs_batch, training=True)
                    prob = tf.nn.softmax(logits)
                    act_onehot = tf.one_hot(act_batch, self.action_dim)
                    logp = tf.reduce_sum(act_onehot * tf.math.log(prob + 1e-8), axis=1)
                    ratio = tf.exp(logp - logp_old)
                    clip_adv = tf.where(adv_batch>0,
                                        (1+self.clip_ratio)*adv_batch,
                                        (1-self.clip_ratio)*adv_batch)
                    policy_loss = -tf.reduce_mean(tf.minimum(ratio*adv_batch, clip_adv))
                grads = tape.gradient(policy_loss, self.actor.trainable_variables)
                self.actor.optimizer.apply_gradients(zip(grads, self.actor.trainable_variables))
                self.critic.train_on_batch(obs_batch, ret_batch)
        # reset buffers
        self._init_buffers()

    def _compute_gae(self):
        rewards = np.array(self.rew_buf + [0])
        values = np.array(self.val_buf + [0])
        dones = np.array(self.done_buf + [0])
        advs = np.zeros_like(rewards[:-1])
        gae = 0
        for t in reversed(range(len(rewards)-1)):
            delta = rewards[t] + self.gamma*values[t+1]*(1-dones[t]) - values[t]
            gae = delta + self.gamma*self.lam*(1-dones[t])*gae
            advs[t] = gae
        rets = advs + values[:-1]
        return rets, advs