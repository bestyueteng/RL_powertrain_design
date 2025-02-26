import gymnasium as gym
import torch as th
from torch import nn
from torch_geometric.nn import GCNConv, SAGEConv, GATConv, global_max_pool, BatchNorm
import torch_geometric as thg
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from gymnasium import spaces
import gymnasium as gym
import torch as th
from torch import nn
from torch_geometric.nn import GCNConv,SAGEConv,GATConv
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
from torch_geometric.utils.convert import from_networkx
from powertrain_topology_rl.utils import *
import networkx as nx
import torch.nn.functional as F

class GraphFeaturesExtractor(BaseFeaturesExtractor):
    """
    Graph feature extractor for Graph observation spaces.
    Build a graph convolutional network for belief state extraction
    
    :param observation_space:
    :param output_dim: Number of features to output from GNN. 
    """
    
    def __init__(self, observation_space: gym.spaces.Box, 
                 gnn_output_dim: int = 64, model='GCN'):
        super().__init__(observation_space, features_dim=gnn_output_dim)
        self._features_dim = gnn_output_dim
        node_feature_num = observation_space.shape[0]
        
        if model == 'GCN':
            self.conv_layer = GCNConv(node_feature_num, 2*gnn_output_dim)
        elif model == 'SAGE':
            self.conv_layer = SAGEConv(node_feature_num, 2*gnn_output_dim)
        elif model == 'GAT':
            self.conv_layer = GATConv(node_feature_num, 2*gnn_output_dim)
        
        self.conv_layer2 = GCNConv(2*gnn_output_dim, 2*gnn_output_dim) # new layer 
        self.linear_layer = nn.Linear(2*gnn_output_dim, gnn_output_dim)
    
    def forward(self, observations: th.Tensor):
        # Check if observations have a batch dimension
        if observations.dim() == 3:
            # Assuming shape: (batch_size, num_nodes, node_features)
            batch_size = observations.size(0)
            features = []
            for i in range(batch_size):
                obs = observations[i]  # Shape: (num_nodes, node_features)
                G = nx.from_numpy_array(obs.cpu().numpy())
                pyg_graph = from_networkx(G)
                pyg_graph = pyg_graph.to(observations.device)
                
                num_nodes = pyg_graph.num_nodes
                # Define node features 'x' as one-hot vectors or use existing features
                x = th.eye(num_nodes, device=observations.device)
                pyg_graph.x = x
                
                edge_index = pyg_graph.edge_index
                h = self.conv_layer(x, edge_index).relu()
                h = F.dropout(h, p=0.2, training=self.training)
                h = self.conv_layer2(h, edge_index).relu()
                h = F.dropout(h, p=0.2, training=self.training)
                h = global_max_pool(h, pyg_graph.batch)
                h = self.linear_layer(h).relu()
                features.append(h)
            
            # Concatenate features from all samples in the batch
            return th.stack(features)
        else:
            # Existing single observation handling
            observation_np = observations.cpu().numpy()
            if observation_np.ndim == 3 and observation_np.shape[0] == 1:
                observation_np = observation_np.squeeze(0)
            G = nx.from_numpy_array(observation_np, create_using=nx.Graph)
            pyg_graph = from_networkx(G)
            pyg_graph = pyg_graph.to(observations.device)
            
            num_nodes = pyg_graph.num_nodes
            x = th.eye(num_nodes, device=observations.device)
            pyg_graph.x = x
            
            x, edge_index, batch = pyg_graph.x, pyg_graph.edge_index, pyg_graph.batch
            h = self.conv_layer(x, edge_index).relu()
            h = F.dropout(h, p=0.2, training=self.training)
            h = self.conv_layer2(h, edge_index).relu()
            h = F.dropout(h, p=0.2, training=self.training)
            h = global_max_pool(h, batch)
            h = self.linear_layer(h).relu()
            return h

class CustomCombinedExtractor_GNN(BaseFeaturesExtractor):
    def __init__(self, observation_space):
        # super().__init__(observation_space, features_dim=1)

        extractors = {}
        total_concat_size = 0

        for key, subspace in observation_space.spaces.items():
            if key == "DSM":
                num_nodes = subspace.shape[0]
                extractors[key] = GraphFeaturesExtractor(subspace)
                total_concat_size += 64  # The final output dim from AdjacencyGCN
            elif key == "Library":
                # Standard MLP for a 1D Box
                extractors[key] = nn.Sequential(
                    nn.Linear(subspace.shape[0], 64),
                    nn.ReLU(),
                    nn.Linear(64, 64),
                    nn.ReLU()
                )
                total_concat_size += 64
                
        super().__init__(observation_space, features_dim=total_concat_size)
        
        self.extractors = nn.ModuleDict(extractors)
        
        self._features_dim = total_concat_size

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))

        # Concatenate all encoded features
        # Ensure that all features are compatible for concatenation
        # **Concatenate all features**
        concatenated = th.cat(encoded_tensor_list, dim=1)  # Shape [batch_size, total_concat_size]

        
        return concatenated
        

class CustomCombinedExtractor_MLP(BaseFeaturesExtractor):
    def __init__(self, observation_space):
        super().__init__(observation_space, features_dim=1)

        extractors = {}
        total_concat_size = 0
        
        for key, subspace in observation_space.spaces.items():
            if key == "DSM":
                extractors[key] = nn.Sequential(
                        nn.Linear(subspace.shape[0], 64),
                        nn.ReLU(),
                        nn.Linear(64, 64),
                        nn.ReLU()
                    )
                total_concat_size += 64
            elif key == "Library":
                # Standard MLP for a 1D Box
                extractors[key] = nn.Sequential(
                    nn.Linear(subspace.shape[0], 32),
                    nn.ReLU(),
                )
                total_concat_size += 32
            elif key == "performance_req":
                extractors[key] = nn.Sequential(
                    nn.Linear(subspace.shape[0], 32),
                    nn.ReLU(),
                )
                total_concat_size += 32

        self.extractors = nn.ModuleDict(extractors)
        
        
        self._features_dim = total_concat_size

    def forward(self, observations) -> th.Tensor:
        encoded_tensor_list = []

        # self.extractors contain nn.Modules that do all the processing.
        for key, extractor in self.extractors.items():
            encoded_tensor_list.append(extractor(observations[key]))

        # Concatenate all encoded features
        # Ensure that all features are compatible for concatenation
        # **Concatenate all features**
        concatenated = th.cat(encoded_tensor_list, dim=1)  # Shape [batch_size, total_concat_size]

        
        return concatenated
