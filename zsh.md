# ZSH Config

> Reference:
> 1. https://holychung.medium.com/%E5%88%86%E4%BA%AB-oh-my-zsh-powerlevel10k-%E5%BF%AB%E9%80%9F%E6%89%93%E9%80%A0%E5%A5%BD%E7%9C%8B%E5%A5%BD%E7%94%A8%E7%9A%84-command-line-%E7%92%B0%E5%A2%83-f66846117921
> 2. https://catalins.tech/zsh-plugins/

Install `zsh`:

```bash
sudo apt install zsh
```

Change login shell:

```bash
chsh -s $(which zsh)
```

Install `oh-my-zsh`:

```bash
wget https://raw.githubusercontent.com/ohmyzsh/ohmyzsh/master/tools/install.sh
sh install.shell
```

Install `powerlevel10k` theme:

```bash
git clone --depth=1 https://github.com/romkatv/powerlevel10k.git ${ZSH_CUSTOM:-$HOME/.oh-my-zsh/custom}/themes/powerlevel10k
```

Then modify `~/.zshrc`:

```bash
ZSH_THEME="powerlevel10k/powerlevel10k"
```

