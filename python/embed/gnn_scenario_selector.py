import os
import pickle

import dgl
import dgl.function as fn
import torch
import torch.nn as nn
import torch.nn.functional as F


class GatedGCNLayer(nn.Module):
    def __init__(self, input_dim, output_dim, dropout, batch_norm, residual=False):
        super().__init__()
        self.in_channels = input_dim
        self.out_channels = output_dim
        self.dropout = dropout
        self.batch_norm = batch_norm
        self.residual = residual

        if input_dim != output_dim:
            self.residual = False

        self.A = nn.Linear(input_dim, output_dim, bias=True)
        self.B = nn.Linear(input_dim, output_dim, bias=True)
        self.C = nn.Linear(input_dim, output_dim, bias=True)
        self.D = nn.Linear(input_dim, output_dim, bias=True)
        self.E = nn.Linear(input_dim, output_dim, bias=True)
        self.bn_node_h = nn.BatchNorm1d(output_dim)
        self.bn_node_e = nn.BatchNorm1d(output_dim)

    def forward(self, g, h, e):

        h_in = h  # for residual connection
        e_in = e  # for residual connection

        g.ndata['h'] = h
        g.ndata['Ah'] = self.A(h)
        g.ndata['Bh'] = self.B(h)
        g.ndata['Dh'] = self.D(h)
        g.ndata['Eh'] = self.E(h)
        g.edata['e'] = e
        g.edata['Ce'] = self.C(e)

        g.apply_edges(fn.u_add_v('Dh', 'Eh', 'DEh'))
        g.edata['e'] = g.edata['DEh'] + g.edata['Ce']
        g.edata['sigma'] = torch.sigmoid(g.edata['e'])
        g.update_all(fn.u_mul_e('Bh', 'sigma', 'm'), fn.sum('m', 'sum_sigma_h'))
        g.update_all(fn.copy_e('sigma', 'm'), fn.sum('m', 'sum_sigma'))
        g.ndata['h'] = g.ndata['Ah'] + g.ndata['sum_sigma_h'] / (g.ndata['sum_sigma'] + 1e-6)
        # g.update_all(self.message_func,self.reduce_func)
        h = g.ndata['h']  # result of graph convolution
        e = g.edata['e']  # result of graph convolution

        if self.batch_norm:
            h = self.bn_node_h(h)  # batch normalization
            e = self.bn_node_e(e)  # batch normalization

        h = F.relu(h)  # non-linear activation
        e = F.relu(e)  # non-linear activation

        if self.residual:
            h = h_in + h  # residual connection
            e = e_in + e  # residual connection

        h = F.dropout(h, self.dropout, training=self.training)
        e = F.dropout(e, self.dropout, training=self.training)

        return h, e

    def __repr__(self):
        return '{}(in_channels={}, out_channels={})'.format(self.__class__.__name__,
                                                            self.in_channels,
                                                            self.out_channels)


class MLPReadout(nn.Module):
    def __init__(self, input_dim, output_dim, L=2):  # L=nb_hidden_layers
        super().__init__()
        list_FC_layers = [nn.Linear(input_dim // 2 ** l, input_dim // 2 ** (l + 1), bias=True) for l in range(L)]
        list_FC_layers.append(nn.Linear(input_dim // 2 ** L, output_dim, bias=True))
        self.FC_layers = nn.ModuleList(list_FC_layers)
        self.L = L

    def forward(self, x):
        y = x
        for l in range(self.L):
            y = self.FC_layers[l](y)
            y = F.relu(y)
        y = self.FC_layers[self.L](y)
        return y


"""
    ResGatedGCN: Residual Gated Graph ConvNets
    An Experimental Study of Neural Networks for Variable Graphs (Xavier Bresson and Thomas Laurent, ICLR 2018)
    https://arxiv.org/pdf/1711.07553v2.pdf
    Slightly adapted from 
    https://github.com/graphdeeplearning/benchmarking-gnns/nets/molecules_graph_regression/gated_gcn_net.py
    and 
    https://github.com/graphdeeplearning/benchmarking-gnns/nets/molecules_graph_regression/layers/mlp_readout_layer.py
    and 
    https://github.com/graphdeeplearning/benchmarking-gnns/nets/molecules_graph_regression/layers/gated_gcn_layer.py
"""


class GatedGCNNet(torch.nn.Module):
    def __init__(self, net_params):
        super().__init__()
        num_node_features = net_params['num_node_features']
        num_edge_features = net_params['num_edge_features']
        hidden_dim = net_params['hidden_dim']
        out_dim = net_params['out_dim']
        in_feat_dropout = net_params['in_feat_dropout']
        dropout = net_params['dropout']
        n_layers = net_params['L']

        self.readout = net_params['readout']
        self.batch_norm = net_params['batch_norm']
        self.residual = net_params['residual']
        self.edge_feat = net_params['edge_feat']
        self.device = net_params['device']
        self.pos_enc = net_params['pos_enc']
        if self.pos_enc:
            pos_enc_dim = net_params['pos_enc_dim']
            self.embedding_pos_enc = nn.Linear(pos_enc_dim, hidden_dim)

        self.embedding_h = nn.Linear(num_node_features, hidden_dim)

        if self.edge_feat:
            self.embedding_e = nn.Linear(num_edge_features, hidden_dim)
        else:
            self.embedding_e = nn.Linear(1, hidden_dim)

        self.in_feat_dropout = nn.Dropout(in_feat_dropout)

        self.layers = nn.ModuleList([GatedGCNLayer(hidden_dim, hidden_dim, dropout,
                                                   self.batch_norm, self.residual) for _ in range(n_layers)])

        self.MLP_layer = MLPReadout(hidden_dim + 1, 1, L=1)

    def forward(self, g, alpha, h, e, h_pos_enc=None):

        # input embedding
        h = self.embedding_h(h)
        h = self.in_feat_dropout(h)
        if self.pos_enc:
            h_pos_enc = self.embedding_pos_enc(h_pos_enc.float())
            h = h + h_pos_enc
        if not self.edge_feat:  # edge feature set to 1
            e = torch.ones(e.size(0), 1).to(self.device)
        e = self.embedding_e(e)

        # convnets
        for conv in self.layers:
            h, e = conv(g, h, e)
        g.ndata['h'] = h

        if self.readout == "sum":
            hg = dgl.sum_nodes(g, 'h')
        elif self.readout == "max":
            hg = dgl.max_nodes(g, 'h')
        elif self.readout == "mean":
            hg = dgl.mean_nodes(g, 'h')
        else:
            hg = dgl.mean_nodes(g, 'h')  # default readout is mean nodes

        return self.MLP_layer(torch.cat((hg, alpha), 1))


def load_gnn(model_path, params_path, use_cpu=False, gpu_id=0):
    net_params = pickle.load(open(params_path, "rb"))

    if use_cpu:
        device = torch.device('cpu')
    else:
        os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
        os.environ["CUDA_VISIBLE_DEVICES"] = str(gpu_id)
        # print(f'cuda available with GPU: {torch.cuda.get_device_name(0)}')
        device = torch.device("cuda")

    model = GatedGCNNet(net_params).to(device)

    # print('GNN model loaded.')

    model.load_state_dict(torch.load(model_path, map_location=device))
    model.eval()

    return [model, device]


def get_graph(scenario, n_tasks, precedence_constraints, mutex_constraints, model_data):
    g = dgl.DGLGraph()
    g.add_nodes(n_tasks)

    nodes_features = []

    for i in range(n_tasks):
        nodes_features.append([model_data["lower_bounds"][scenario][i]])

    g.ndata["feat"] = torch.Tensor(nodes_features)

    edge_features = []

    for p in precedence_constraints:
        i = p[0]
        j = p[1]

        g.add_edge(i, j)

        edge_features.append([0, model_data["precedence"][scenario][(i, j)]])

    for m in mutex_constraints:
        i = int(m[0])
        j = int(m[1])

        g.add_edge(i, j)
        g.add_edge(j, i)

        edge_features.append([1, model_data["mutex"][scenario][(i, j)]])

        edge_features.append([1, model_data["mutex"][scenario][(j, i)]])

    g.edata["feat"] = torch.Tensor(edge_features)

    return g


def get_predictions(model, device, n_tasks, n_scenarios, alpha, precedence_constraints,
                    mutex_constraints, data_dict, use_cpu=False, gpu_id=0):
    """
    :param model
    :param device
    :param n_tasks: number of tasks
    :param n_scenarios: number of scenarios to predict
    :param alpha: alpha-robustness value
    :param precedence_constraints: list of pairs (i,j) of precedence constraints (the original ones - not
    containing the extra ones added to the milp)
    :param mutex_constraints: list of pairs (i, j) of mutex constraints (single pair for each constraint)
    :param data_dict: dictionary structured as shown in the get_graph function, gnn_data_handler.py
    :param use_cpu: True iff using cpu
    :param gpu_id: gpu_id in case !use_cpu
    :return: a list of probabilities, one for each scenario (1 = definitely pick, 0 = definitely not pick)
    """
    dgl_graphs = list(map(lambda scenario: get_graph(scenario, n_tasks, precedence_constraints, mutex_constraints,
                                                     data_dict), range(n_scenarios)))

    batch_graphs = dgl.batch(dgl_graphs).to(device)

    batch_x = batch_graphs.ndata['feat'].to(device)
    batch_e = batch_graphs.edata['feat'].to(device)

    batch_alphas = torch.tensor([[alpha] for _ in range(n_scenarios)]).to(device)

    with torch.no_grad():
        predictions = model(batch_graphs, batch_alphas, batch_x, batch_e)

    predictions = nn.Sigmoid()(predictions).squeeze().tolist()

    # print(predictions)

    return predictions


def get_predictions_graph(model, device, n_scenarios, alpha, graphs, use_cpu, gpu_id):
    """
    :param model
    :param device
    :param n_scenarios: number of scenarios to predict
    :param alpha: alpha-robustness value
    :param graphs: list of heterogeneous graph used as the into to the model
    :param use_cpu: True iff using cpu
    :param gpu_id: gpu_id in case !use_cpu

    :return: a list of probabilities, one for each scenario (1 = definitely pick, 0 = definitely not pick)
    """
    batch_graphs = dgl.batch(graphs).to(device)
    batch_x = batch_graphs.ndata['feat'].to(device)
    batch_e = batch_graphs.edata['feat'].to(device)
    batch_alphas = torch.tensor([[alpha] for _ in range(n_scenarios)], device=device)

    predictions = model(batch_graphs, batch_alphas, batch_x, batch_e)
    return nn.Sigmoid()(predictions).squeeze().tolist()