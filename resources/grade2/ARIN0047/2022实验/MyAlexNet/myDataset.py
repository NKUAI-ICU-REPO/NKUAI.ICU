from torch.utils.data import Dataset
import cv2
import numpy as np
from PIL import Image
from torchvision import transforms


class ImageNet2012(Dataset):
    def __init__(self, datapah):
        super(ImageNet2012, self).__init__()

        self.datapath = datapah

        namef = open(datapah + 'train_label.txt', 'r')

        self.names = namef.readlines()
        self.name = []
        for i in range(len(self.names)):
            nameimg, id = self.names[i].split()
            if int(id) < 20:
                self.name.append(self.names[i])

        self.len = len(self.name)

        self.transforms = transforms.Compose(
            [transforms.ToTensor(),
             transforms.Normalize(mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5])]
        )

    def __len__(self):
        return self.len

    def __getitem__(self, item):
        name = self.name[item]

        name, id = name.split()

        img = Image.open(self.datapath + 'ILSVRC2012_img_train/' + name)
        img = img.convert('RGB')

        id = float(id)

        img = img.resize((224, 224))

        img = self.transforms(img)

        return img, id
