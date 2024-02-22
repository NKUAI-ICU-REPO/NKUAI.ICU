import torch.optim
from torch.utils.data import DataLoader
import torch.nn as nn
from torch.autograd import Variable

from AlexNet import alexnet
from myDataset import ImageNet2012

datapath = 'E:/BaiduNetdiskDownload/ImageNet 2012 DataSets/'

trainDataset = ImageNet2012(datapath)

trainData = DataLoader(dataset=trainDataset, batch_size=32, shuffle=True, num_workers=2)

Alexnet = alexnet(20)
#Alexnet.load_state_dict(torch.load('Alexnet.pth'))
Alexnet.cuda()

lossFun = nn.CrossEntropyLoss()
optimizer = torch.optim.Adam(params=Alexnet.parameters(), lr=1e-4)

if __name__ == "__main__":

    Epochs = 100
    L = len(trainData)
    for epoch in range(Epochs):
        for i, (img, id) in enumerate(trainData):
            img = img.cuda()
            id = id.cuda()

            img = Variable(img, requires_grad=True)
            id = Variable(id, requires_grad=True)

            output = Alexnet(img)
            loss = lossFun(output, id.long())

            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

            iter = epoch * L + i + 1
            if iter % 10 == 0:
                print('epoch:{},iter:{},loss:{:.6f},remain:{}'.format(epoch + 1, iter, loss, L - i - 1))

        torch.save(Alexnet.state_dict(), 'Alexnet.pth')





