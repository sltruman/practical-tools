from .constants import RotationType, Axis
from .auxiliary_methods import intersect, set_to_decimal
import numpy as np
import copy

DEFAULT_NUMBER_OF_DECIMALS = 2
PLANE_SAMPLE_RATE = 100
START_POSITION = [0, 0, 0]
MINIMUM_INTERVAL = 0.1


class Item:
    def __init__(self, name, width, height, depth, weight) -> None:
        self.name = name
        self.width = width
        self.height = height
        self.depth = depth
        self.weight = weight
        self.rotation_type = 0
        self.position = START_POSITION
        self.number_of_decimals = DEFAULT_NUMBER_OF_DECIMALS

        # self.center_position = [self.position[0] + self.get_dimension()[0]/2, self.position[1] + self.get_dimension()[1]/2]

        self.volume = self.width * self.height * self.depth

    def format_numbers(self, number_of_decimals=DEFAULT_NUMBER_OF_DECIMALS):
        self.width = set_to_decimal(self.width, number_of_decimals)
        self.height = set_to_decimal(self.height, number_of_decimals)
        self.depth = set_to_decimal(self.depth, number_of_decimals)
        self.weight = set_to_decimal(self.weight, number_of_decimals)
        self.position[0] = set_to_decimal(self.position[0], number_of_decimals)
        self.position[1] = set_to_decimal(self.position[1], number_of_decimals)
        self.position[2] = set_to_decimal(self.position[2], number_of_decimals)

        self.number_of_decimals = number_of_decimals

    def get_dimension(self):
        return [self.width, self.height, self.depth]



class Layer:
    def __init__(self, num, bin_size, bin_fitted_items, depth_map) -> None:
        self.bin_size = bin_size
        self.num = num
        self.bin_fitted_items = bin_fitted_items
        self.layer_fitted_items = []
        self.depth_map = depth_map
        self.previous_layer_depth_map = copy.deepcopy(depth_map)


    def update_depth_map(self, item):
        dimension = item.get_dimension()
        min_maps_index = [int(PLANE_SAMPLE_RATE*item.position[0]), int(PLANE_SAMPLE_RATE*item.position[1])]
        max_maps_index = [int(PLANE_SAMPLE_RATE*(item.position[0]+dimension[0])), int(PLANE_SAMPLE_RATE*(item.position[1]+dimension[1]))]
        self.depth_map[min_maps_index[0]:max_maps_index[0], min_maps_index[1]:max_maps_index[1]] = np.float64(dimension[2]+item.position[2])

    def adjust_item_position(self, item, pivot):
        item.position = pivot
        dimension = item.get_dimension()
        if (self.bin_size[0] < item.position[0] + dimension[0] or
            self.bin_size[1] < item.position[1] + dimension[1] or
            self.bin_size[2] < item.position[2] + dimension[2]):
            return False

        maps_index = [int(PLANE_SAMPLE_RATE*(pivot[0]+dimension[0]/2)), int(PLANE_SAMPLE_RATE*(pivot[1]+dimension[1]/2))]
        item.position[2] = set_to_decimal(self.previous_layer_depth_map[maps_index[0], maps_index[1]], item.number_of_decimals)

        # maps_index = [int(PLANE_SAMPLE_RATE*(pivot[0]+dimension[0]/2)), int(PLANE_SAMPLE_RATE*(pivot[1]+dimension[1]/2))]
        # min_maps_index = [int(PLANE_SAMPLE_RATE*item.position[0]), int(PLANE_SAMPLE_RATE*item.position[1])]
        # max_maps_index = [int(PLANE_SAMPLE_RATE*(item.position[0]+dimension[0])), int(PLANE_SAMPLE_RATE*(item.position[1]+dimension[1]))]
        # higgest_depth = np.max(self.previous_layer_depth_map[min_maps_index[0]:max_maps_index[0], min_maps_index[1]:max_maps_index[1]])
        # item.position[2] = set_to_decimal(higgest_depth, item.number_of_decimals)
        # if higgest_depth - self.previous_layer_depth_map[maps_index[0], maps_index[1]] > MINIMUM_INTERVAL:
        #     return False
        return True

    def detect_collision(self, item):
        dimension = item.get_dimension()
        if (self.bin_size[0] < item.position[0] + dimension[0] or
            self.bin_size[1] < item.position[1] + dimension[1] or
            self.bin_size[2] < item.position[2] + dimension[2]):
            return False
        
        for current_item_in_bin in self.bin_fitted_items:
            if intersect(current_item_in_bin, item):
                return False

        min_maps_index = [int(PLANE_SAMPLE_RATE*item.position[0]), int(PLANE_SAMPLE_RATE*item.position[1])]
        max_maps_index = [int(PLANE_SAMPLE_RATE*(item.position[0]+dimension[0])), int(PLANE_SAMPLE_RATE*(item.position[1]+dimension[1]))]
        if (self.depth_map[min_maps_index[0]:max_maps_index[0], min_maps_index[1]:max_maps_index[1]] <= np.float64(item.position[2])).all():
            return True
        else:
            return False

    def put_item(self, item, pivot):
        fitted = False

        # if self.get_total_weight() + item.weight > self.max_weight:
        #     fitted = False
        #     return fitted

        original_position = item.position

        for i in range(0, len(RotationType.ALL)):
            item.rotation_type = i
            success = self.adjust_item_position(item, pivot)

            if success:
                fitted = self.detect_collision(item)
            else:
                fitted = False

            if fitted:
                # print("success to put item {0} in layer {1}:".format(item.name, self.num))
                self.update_depth_map(item)
                self.layer_fitted_items.append(item)
                break
            
        
        if not fitted:
            item.position = original_position
            # item.rotation_type = 0

        return fitted


    def pack_to_layer(self, previous_layer, item):
        fitted = False
        
        # the first layer
        if previous_layer == None:
            # the first item to put
            if not self.layer_fitted_items:
                return self.put_item(item, START_POSITION)
            
            # the others items
            # fitted = self.choose_position(item)
            for axis in range(0, 2):
                items_in_layer = self.layer_fitted_items
                for iil in items_in_layer:
                    pivot = [0, 0, 0]
                    w, h, d = iil.get_dimension()
                    if axis == Axis.WIDTH:
                        pivot = [
                            iil.position[0] + w,
                            iil.position[1],
                            iil.position[2]
                        ]
                    elif axis == Axis.HEIGHT:
                        pivot = [
                            iil.position[0],
                            iil.position[1] + h,
                            iil.position[2]
                        ]

                    if self.put_item(item, pivot):
                        fitted = True
                        break    
                if fitted:
                    break
            
            return fitted


        # other layers
        if not self.layer_fitted_items:
            # pack the first item in this layer
            for base_item in previous_layer.layer_fitted_items:
                pivot = [0, 0, 0]
                w, h, d = base_item.get_dimension()
                pivot = [base_item.position[0],
                        base_item.position[1],
                        base_item.position[2] + d]

                if self.put_item(item, pivot):
                    fitted = True
                    break 
            return fitted

        else:
            # according the item in this layer, pack the leftover unfitted items
            for axis in range(0, 2):
                items_in_layer = self.layer_fitted_items
                for iil in items_in_layer:
                    pivot = [0, 0, 0]
                    w, h, d = iil.get_dimension()
                    if axis == Axis.WIDTH:
                        pivot = [
                            iil.position[0] + w,
                            iil.position[1],
                            iil.position[2]
                        ]
                    elif axis == Axis.HEIGHT:
                        pivot = [
                            iil.position[0],
                            iil.position[1] + h,
                            iil.position[2]
                        ]

                    if self.put_item(item, pivot):
                        fitted = True
                        return fitted
            
            if not fitted:
                # try the z axis of the items in previous layer
                for base_item in self.bin_fitted_items:
                    pivot = [0, 0, 0]
                    w, h, d = base_item.get_dimension()
                    pivot = [base_item.position[0],
                            base_item.position[1],
                            base_item.position[2] + d]

                    if self.put_item(item, pivot):
                        fitted = True
                        return fitted

       


class Bin:
    def __init__(self, name, width, height, depth, max_weight) -> None:
        self.name = name
        self.width = width
        self.height = height
        self.depth = depth
        self.max_weight = max_weight
        self.volume = self.width * self.height * self.depth

        self.layer_num = 0
        self.layers = []

        # self.items = []

        self.bin_fitted_items = []
        self.bin_unfitted_items = []

        self.number_of_decimals = DEFAULT_NUMBER_OF_DECIMALS

    def format_numbers(self, number_of_decimals):
        self.width = set_to_decimal(self.width, number_of_decimals)
        self.height = set_to_decimal(self.height, number_of_decimals)
        self.depth = set_to_decimal(self.depth, number_of_decimals)
        self.max_weight = set_to_decimal(self.max_weight, number_of_decimals)
        self.number_of_decimals = number_of_decimals

    def get_total_weight(self):
        total_weight = 0

        for item in self.bin_fitted_items:
            total_weight += item.weight

        return total_weight

    def compare(self, unfitted_items):
        if not unfitted_items:
            return True
        if len(self.bin_unfitted_items) != len(unfitted_items):
            return False
        for idx in range(len(self.bin_unfitted_items)):
            if self.bin_unfitted_items[idx].name != unfitted_items[idx].name:
                return False
        return True

    def pack_to_bin(self, items):
        self.bin_unfitted_items = items
        
        # # create layers for packing
        # if not self.layers:
        #     # create the first layer for packing
        #     self.layers.append(Layer(self.layer_num, self.bin_unfitted_items))
        #     self.layer_num += 1
        #     # try to put the item in the layer and 
        
        depth_map = np.zeros((int(PLANE_SAMPLE_RATE * self.width)+1, int(PLANE_SAMPLE_RATE * self.height)+1))

        while True:
            unfitted_items = []
            
            # the first layer
            if not self.layers:
                print("################# create the layer [{0}] #################".format(self.layer_num))
                self.layers.append(Layer(self.layer_num, [self.width, self.height, self.depth], self.bin_fitted_items, depth_map))
                for item in self.bin_unfitted_items:
                    # print("the number of fitted item: ", len(self.bin_fitted_items))
                    # put the item in this layer
                    success = self.layers[self.layer_num].pack_to_layer(None, item)
                    if success:
                        print("successfully placed item [{0}] on layer [{1}]:".format(item.name, self.layer_num))
                        self.bin_fitted_items.append(item)
                    else:
                        print("fail to place item [{0}] in layer [{1}]:".format(item.name, self.layer_num))
                        unfitted_items.append(item)
                # complete the packing or not
                if self.compare(unfitted_items):
                    break
                self.bin_unfitted_items = copy.deepcopy(unfitted_items)
                unfitted_items = []
                self.layer_num += 1


            # other layers
            print("################# create the layer [{0}] #################".format(self.layer_num))
            self.layers.append(Layer(self.layer_num, [self.width, self.height, self.depth], self.bin_fitted_items, self.layers[self.layer_num-1].depth_map))
            
            for item in self.bin_unfitted_items:
                # put the item in this layer
                success = self.layers[self.layer_num].pack_to_layer(self.layers[self.layer_num-1], item)
                if success:
                    print("successfully placed item [{0}] on layer [{1}]:".format(item.name, self.layer_num))
                    self.bin_fitted_items.append(item)
                else:
                    print("fail to place item [{0}] in layer [{1}]:".format(item.name, self.layer_num))
                    unfitted_items.append(item)

            # continue or not
            if self.compare(unfitted_items):
                break
            # update the unfitted items
            self.bin_unfitted_items = copy.deepcopy(unfitted_items)
            self.layer_num += 1


class Packer:
    def __init__(self) -> None:
        self.bins = []
        self.layers = []
        self.items = []
        self.unfit_items = []
        self.total_num_items = 0
    

    def add_bin(self, bin):
        return self.bins.append(bin)
    

    def add_item(self, item):
        self.total_items = len(self.items) + 1

        return self.items.append(item)


    def pack(self, bigger_first=False, distribute_items=False, number_of_decimals=DEFAULT_NUMBER_OF_DECIMALS):
        
        for bin in self.bins:
            bin.format_numbers(number_of_decimals)

        for item in self.items:
            item.format_numbers(number_of_decimals)

        self.bins.sort(
            key=lambda bin: bin.volume, reverse=bigger_first
        )
        # self.items.sort(
        #     key=lambda item: item.volume, reverse=bigger_first
        # )
        
        for bin in self.bins:
            bin.pack_to_bin(self.items)

            print("the bin fitted items number: ", len(bin.bin_fitted_items))
            
            # self.pack_to_bin(bin)
                    
            # if np.max(self.bins.depth_map) >= bin.depth:
            #     break




