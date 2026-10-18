import torch
from torch import nn
class AdaptiveForecast(nn.Module):
    def __init__(self):
        super().__init__();self.encoder=nn.GRU(44,24,batch_first=True);self.transition=nn.GRUCell(10,24);self.head=nn.Sequential(nn.Linear(24,24),nn.SiLU(),nn.Linear(24,6))
    def latent(self,x,actions):
        z=self.encoder(x)[0][:,-1];out=[]
        for k in range(actions.shape[1]):
            z=self.transition(torch.cat([actions[:,k],torch.full_like(actions[:,k,:1],.05)],-1),z)
            if k in (1,3,7):out.append(z)
        return torch.stack(out,1)
    def forward(self,x,actions):return self.head(self.latent(x,actions))
def noncollapse(z):
    z=z.reshape(-1,24);directions=torch.randn(24,16,device=z.device);directions/=directions.norm(dim=0,keepdim=True).clamp_min(1e-8);grid=torch.linspace(0,3,17,device=z.device);phase=(z@directions)[...,None]*grid;g=torch.exp(-.5*grid**2)
    return (2*torch.trapezoid(((phase.cos().mean(0)-g)**2+phase.sin().mean(0)**2)*g,grid,dim=-1)).mean()
def latent_loss(m,x,u,xf):
    z=m.latent(x,u);target=m.encoder(xf.reshape(-1,16,44))[0][:,-1].reshape(len(x),3,24)
    return (z-target).square().mean()+.1*(noncollapse(z)+noncollapse(target))
