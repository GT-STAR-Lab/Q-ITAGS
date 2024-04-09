from sklearn.model_selection import train_test_split
import numpy as np
from abc import ABC, abstractmethod


class QueryableExpert(ABC):
    @abstractmethod
    def get_query_label(self, features):
       pass

