import tensorflow as tf
from tensorpack import *
from tensorpack.tfutils.summary import add_moving_summary
from tensorpack import ModelDesc
from tensorpack import InputDesc
from tensorpack import Callback
from tensorpack import regularize_cost
from tensorpack import BatchNorm, Conv2D, Deconv2D
import cv2
import numpy as np

INPUT_SHAPE = 128

########## Joans ##########
LeakyRelu = tf.nn.leaky_relu
########## Joans ##########

def conv(name, l, channel, k, stride=1, net=None, use_bn=True):
    with tf.variable_scope(name):
        if use_bn:
            l = BatchNorm('bn', l)
            l = LeakyReLU('leak', l, 0.33)

        if stride > 1:
            l = Conv2D('conv', l, channel, k, stride=stride)
        else:
            l = tf.layers.conv2d(l, channel, k, padding='SAME', dilation_rate=2)

        if net is not None:
            net[name] = l

    return l


def deconv(name, l, k, net=None):
    with tf.variable_scope(name):
        #l = tf.layers.UpSampling2D(l,2)
        l = BatchNorm('ln', l)
        l = LeakyReLU('leak', l, 0.33)
        l = Deconv2D(name, l, k, 5, stride=2)

        if net is not None:
            net[name] = l

    return l


def refine(name, l, k, skip, net):
    with tf.variable_scope(name):
        if l.shape[1] != skip.shape[1] or l.shape[2] != skip.shape[2]:
            h_pad = int(l.shape[1]) - int(skip.shape[1])
            w_pad = int(l.shape[2]) - int(skip.shape[2])

            skip = tf.pad(skip, ((0, 0), (0, h_pad), (0, w_pad), (0, 0)))

        l = tf.concat([l, skip], -1)
        l = deconv(name, l, k, net)

    return l

def feedforward(image, k=4, scope='Network'):
    net = dict()
    image = tf.to_float(image)
    shape = tf.shape(image)[1:3]
    l = image
    l = conv('conv1', l, k, 7, 2, net, use_bn = False)   # input values should not be messed with
    l = conv('conv1_1', l, k, 7, 1, net)
    l = conv('conv2', l, 2 * k, 7, 2, net)
    l = conv('conv2_1', l, 2 * k, 7, 1, net)
    l = conv('conv3', l, 4 * k, 7, 2, net)
    l = conv('conv3_1', l, 4 * k, 5, 1, net)
    l = conv('conv4', l, 8 * k, 5, 2, net)
    l = conv('conv4_1', l, 8 * k, 5, 1, net)
    l = conv('conv5', l, 8 * k, 3, 2, net)
    l = conv('conv5_1', l, 8 * k, 3, 1, net)
    l = conv('conv6', l, 16 * k, 3, 2, net)
    l = conv('conv6_1', l, 16 * k, 3, 1, net)
    l = conv('conv7', l, 32 * k, 3, 2, net)
    l = conv('conv7_1', l, 32 * k, 3, 1, net)

    l = refine('deconv6', l, 16 * k, net['conv7_1'], net)
    l = refine('deconv5', l, 8 * k, net['conv6_1'], net)
    l = refine('deconv4', l, 4 * k, net['conv5_1'], net)
    l = refine('deconv3', l, 2 * k, net['conv4_1'], net)
    l = refine('deconv2', l, k, net['conv3_1'], net)
    l = refine('deconv1', l, k, net['conv2_1'], net)
    l = tf.image.resize_bilinear(l, shape, name='resize')
    l = tf.concat([l, image], -1, name='concat')
    logits = conv('logit',l,1,3,1,net)    
    prob = tf.sigmoid(logits, name = 'prob')

    return logits, prob, net

class Network(ModelDesc):
    def _get_inputs(self):
        return [InputDesc(tf.float32, [None, INPUT_SHAPE, INPUT_SHAPE, 2], 'input_image'),
                InputDesc(tf.float32, [None, INPUT_SHAPE, INPUT_SHAPE, 1], 'input_label')]

    def _build_graph(self, inputs):
        image, label = inputs
        logits, prob, _ = feedforward(image)
        
        # Loss.
        l2_loss = tf.losses.sigmoid_cross_entropy(tf.to_float(label), logits)
        self.cost = l2_loss

        add_moving_summary(l2_loss)
	
    def _get_optimizer(self):
        learn_rate_op = tf.Variable(1e-3, trainable=False, name='learning_rate')
        tf.summary.scalar('learn rate', learn_rate_op)
        optimizer_op = tf.train.AdamOptimizer(learn_rate_op)
        return optimizer_op


