import lasagne
import numpy as np
import pickle
import skimage.transform
import scipy
import theano
import theano.tensor as T
from lasagne.utils import floatX
import matplotlib.pyplot as plt

from lasagne.layers import *
from lasagne.layers.dnn import Conv2DDNNLayer as ConvLayer
from lasgane.layers import Pool2DLayer as PoolLayer

IMAGE_W = 600
#LAYERNAMES
INPUT = 'input'
CONV1_1 = 'conv1_1'
CONV1_2 = 'conv1_2'
POOL1 = 'pool1'
CONV2_1 = 'conv2_1'
CONV2_2 = 'conv2_2'
POOL2 = 'pool2'
CONV3_1 = 'conv3_1'
CONV3_2 = 'conv3_2'
CONV3_3 = 'conv3_3'
CONV3_4 = 'conv3_4'
POOL3 = 'pool3'
CONV4_1 = 'conv4_1'
CONV4_2 = 'conv4_2'
CONV4_3 = 'conv4_3'
CONV4_4 = 'conv4_4'
POOL4 = 'pool4'
CONV5_1 = 'conv5_1'
CONV5_2 = 'conv5_2'
CONV5_3 = 'conv5_3'
CONV5_4 = 'conv5_4'
POOL5 = 'pool5'

def build_model():
    net = {}
    net[INPUT] = InputLayer(1,3, IMAGE_W, IMAGE_W)
    net[CONV1_1] = ConvLayer(net[INPUT], 64,3, pad=1)
    net[CONV1_2] = ConvLayer(net[CONV1_1], 64, 3, pad=1)
    net[POOL1] = PoolLayer(net[CONV1_2],2, mode='average_exc_pad')
    net[CONV2_1] = ConvLayer(net[POOL1], 128, 3, pad=1)
    net[CONV2_2] = ConvLayer(net[CONV2_1], 128, 3, pad=1)
    net[POOL2] = PoolLayer(net[CONV2_2], 2, mode='average_exc_pad')
    net[CONV3_1] = ConvLayer(net[POOL2], 256, 3, pad=1)
    net[CONV3_2] = ConvLayer(net[CONV3_1], 256, 3, pad=1)
    net[CONV3_3] = ConvLayer(net[CONV3_2], 256, 3, pad=1)
    net[CONV3_4] = ConvLayer(net[CONV3_3], 256, 3, pad=1)
    net[POOL3] = PoolLayer(net[CONV3_4], 2, mode='average_exc_pad')
    net[CONV4_1] = ConvLayer(net[POOL3], 512, 3, pad=1)
    net[CONV4_2] = ConvLayer(net[CONV3_1], 512, 3, pad=1)
    net[CONV4_3] = ConvLayer(net[CONV3_2], 512, 3, pad=1)
    net[CONV4_4] = ConvLayer(net[CONV3_3], 512, 3, pad=1)
    net[POOL4] = PoolLayer(net[CONV4_4], 2, mode='average_exc_pad')
    net[CONV5_1] = ConvLayer(net[POOL4], 512, 3, pad=1)
    net[CONV5_2] = ConvLayer(net[CONV4_1], 512, 3, pad=1)
    net[CONV5_3] = ConvLayer(net[CONV4_2], 512, 3, pad=1)
    net[CONV5_4] = ConvLayer(net[CONV4_3], 512, 3, pad=1)
    net[POOL5] = PoolLayer(net[CONV5_4], 2, mode='average_exc_pad')
    return net


def loadModelWeights():
    net = build_model()
    values = pickle.load(open('pickledmodelweights.pk1'))['param values']
    lasagne.layers.set_all_param_values(net['pool5'], values)


def prep_image(im):
    if len(im.shape) == 2:
        im = im[:,:, np.newaxis]
        im = np.repeat(im, 3, axis=2)
    h, w, _ = im.shape
    if h < w :
        im = skimage.transform.resize(im, (IMAGE_W, w * IMAGE_W/h), preserve_range = True)
    else :
        im = skimage.transform.resize(im, (h * IMAGE_W/w,IMAGE_W), preserve_range = True)
# central crop
        h,w,_ = im.shape
    im = im[h//2-IMAGE_W//2:h//2+IMAGE_W//2, w//2-IMAGE_W//2:w//2+IMAGE_W//2]
    rawim = np.copy(im).astype('uint8')

# shuffle acces to c01
    im = np.swapaxes(np.swapaxes(im,1,2),0,1)
# convert image rgb to bgr
    im = im[::-1, :, :]
    im = im - MEAN_VALUES
    return rawim, floatX(im[np.newaxis])


def loadandPrepImage(imname = 'Tuebingen_Neckarfront.jpg'):
    photo = plt.imread(imname)
    rawim, photo = prep_image(photo)
    plt.imshow(rawim)
    return rawim, photo

art = plt.imread('1920px-VanGogh_-_Starry_Night_-_Google_Art_Project.jpg')
rawimage,art = prep_image(arg)
plt.imshow(rawim)


def gram_matrix(x):
    x = x.flatten(ndim=3)
    g = T.tensor(x,x,axes=([2],[2]))
    return g

def content_loss(P, X, layer):
    p = P[layer]
    x = X[layer]
    loss = 1./2 * ((x-p)**2).sum()
    return loss

def style_loss(A,X,layer):
    a = A[layer]
    x = X[layer]
    A = gram_matrix(a)
    G = gram_matrix(x)

    N = a.shape[1]
    M = a.shape[2]*a.shape[3]

    loss = 1./(4* N ** 2 * M ** 2) * ((G - A)** 2).sum()
    return loss


def total_variation_loss(x):
    return (((x[:,:,:-1,:-1] - x[:,:,1:,:-1])**2 +
            ((x[:,:,:-1,:-1] - x[:,:,:,:-1]))**2 )**1.25).sum()

def getlayers():
  layernames = [CONV4_2, CONV1_1, CONV2_1, CONV3_1, CONV4_1, CONV5_1]
  layers = {k : net[k] for k in layernames}
  return layers

# precompute layer activations for photo and artwork
input_im_theano = T.tensor4()
outputs  = lasagne.layers.get_output(layers.values(), input_im_theano)
photo_features = {k: theano.shared(output.eval({input_im_theano: photo})) for k, output in zip(layers.keys(), outputs)}

art_features = {k : theano.shared(output,eval({input_im_theano: art})) for k, output in zip(layers.keys(), output)}

# Get expression for layer activations for generateed image
generated_image = theano.shared(floatX(np.random.uniform(-128, 128, (1,3, IMAGE_W, IMAGE_W))))

generated_features = lasagne.layers.get_output(layers.values(), generated_image)

gen_features = {k: v for k, v in zip(layers.keys(), gen_features)}


#Define  function
losses = []
#content losss
losses.append(0.001 * content_loss(photo_features, gen_features, CONV4_2))

# style loss
losses.append(0.2e6 * style_loss(art_features, gen_features, CONV1_1))
losses.append(0.2e6 * style_loss(art_features, gen_features, CONV2_1))
losses.append(0.2e6 * style_loss(art_features, gen_features, CONV3_1))
losses.append(0.2e6 * style_loss(art_features, gen_features, CONV4_1))
losses.append(0.2e6 * style_loss(art_features, gen_features, CONV5_1))

# total variation penalty

losses.append(0.1e-7 * total_variation_loss(generated_image))

total_loss=sum(losses)

grad = T.grad(total_loss, generated_image)


# Theano functions to evaluyate loss and gradient

f_loss = theano.function([], total_loss)
f_grad = theano.function([], grad)

def eval_loss(x0):
    x0 = floatX(xo.reshape((1,3,IMAGE_W, IMAGE_W)))
    generated_image.set_value(x0)
    return f_loss().astype('float64')

def eval_grad(x0):
    x0 = floatX(x0.reshape((1,3,IMAGE_W,IMAGE_W)))
    generated_image.set_value(x0)
    return np.array(f_grad()).flatten().astype('float64')

#Initilize with a noise image

generated_image.set_value(floatX(np.random.uniform(-128, 128, (1,3,IMAGE_W, IMAGE_W))))
x0 = generated_image.get_value().astype('float64')
xs = []
xs.append(x0)

#optimize saving the result periodically
for i in range(8):
    print(i)
    scipy.optimize.fmin_l_bfgs_b(eval_loss, x0.flatten(),fprime=eval_grad, maxfun=40)
    x0 = generated_image.get_value().astype('float64')
    xs.append(x0)

def deprocess(x):
    x = np.copy(x[0])
    x += MEAN_VALUES

    x = x[::-1]
    x = np.clip(x, 0, 255).astype('uint8')

    return x

plt.figure(figsize=(12,12))
for i in range(9):
    plt.subplot(3,3, i +1)
    plt.gca().xaxis.set_visible(False)
    plt.gca().yaxis.set_visible(False)
    plt.imshow(deprocess(xs[i]))
plt.tight_layout()










num_inputs, num_units, num_classes = 10, 12, 5

l_inp = InputLayer((None, None, num_inputs))
batchsize, seqlen, _ = l_inp.input_var.shape
l_lstm = LSTMLayer(l_inp, num_units = num_units)
l_shp = ReshapeLayer(l_lstm,(-1, num_units) )
l_dense = DenseLayer(l_shp, num_units=num_classes)
l_out = ReshapeLayer(l_dense,(batchsize, seqlen, num_classes) )
