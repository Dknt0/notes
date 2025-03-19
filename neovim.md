Neovim Note
===

> `Linux`, `Rust`, `neovim`!!!
> 
> Dknt 2025.1

# 1 Vim Basic

**Basic Commands**

`i`: Insert mode.

`dd`: Delete line.

`a`: Append mode.

`A`: Append at the end of line.

`r`: Replace character.

`R`: Replace multiple characters.

`o`: Open a new line below.

`O`: Open a new line above.

`x`: Delete character.

`X`: Delete character before.

`u`: Undo.

`U`: Undo all changes in the current line.

`<C-r>`: (Ctrl-r) Redo.

`<C-g>`: (Ctrl-g) Show the current position of the cursor.

`<C-o>`: (Ctrl-o) Jump back to the previous position.

`<C-i>`: (Ctrl-i) Jump forward to the next position. This shortcup may be redirected by terminal as `<Tab>`.

`p`: Put the last deleted character or text after the cursor.

`P`: Put the last deleted character or text before the cursor.

`v`: Visual mode.

`=` in Visual mode: Align the text.

`y`: Copy text. This can be combined with `yw`.

`p`: Paste text.

`:so`: Run the lua script.

`f`: Jump to.

`F`: Jump to pervious.

`gcc`: Comment/uncomment a single line.

`gc`: Comment/uncomment in visual mode.

`gbc`: Comment/uncomment a block of text. 

**Operator**

`d`: Delete.

`c`: Change.

**Motion**

`w`: to the start of next word.

`b`: to the beginning of the previous word.

`$`: to the end of the line.

`0`: to the beginning of the line.

`e`: to the end of the current word.

`ge`: to the end of the previous word.

`W`, `B`, `E`: motions to WORD.

`^`: to the first non-blank character of the line.

`g_`: to the last non-blank character of the line.

**Search**

`/`: search forward.

`?`: search backward.

`%`: matching parenthesis search.

`n`: next match.

`N`: previous match.

`set ic`: Set the "Ignore Case" option.

`set hls`: Set the "Highlight Search" option.

`set is`: Set the "Incremental Search" option.

**Commands**

Substitute: `:s/old/new/` substitute the first match. `:s/old/new/g` substitute all match in the line. `:#,#s/old/new/g` change every occurrence of a string between to lines (# is line number to be filled). `:%s/old/new/g` change every occurrence in the file. `:%s/old/new/gc` change with a prompt whether to substitute or not.

`:!command`: to execuate an external command.

`:r FILENAME`: to retrieve the context of a file.

**Windows and Tabs**

`<C-w>`: Enter window mode. `h, j, k, l` go to another window. `<, >, -, +` resize the window. `s, v` split the window. `q` quit a window. `<C-w>` (double Ctrl-w) jump from on to another.

`:tabnew`: Open a new tab.

`gt`, `gT`: Navigate between tabs.

`:tabc`: Close a tab.

# 2 Plugins

## `NvChad`

`NvChad` is a `neovim` base configuration. See [NvChad](https://nvchad.com/) for more details.

`NvChad` depends on `neovim` version higher than 0.10. On Ubuntu 24.04, the version of `neovim` installed via apt is 0.9.4, you can install the `AppImage` from `neovim` Github Releases, or install it via `snap`, which provides version 0.10.4. (Till 2025.1.31)

```bash
# Install neovim 0.10.4 via snap
sudo snap install neovim --classic
```

Then you should install **Nerd Font** for normal display of icons. Find **JetbrainsMono Nerd Font** on [Nerd Fonts](https://www.nerdfonts.com/font-downloads). Download it and install it.

```bash
curl -L https://github.com/ryanoasis/nerd-fonts/releases/download/v3.3.0/JetBrainsMono.zip >> JetBrainsMono.zip
unzip JetBrainsMono.zip -d JetBrainsMono
mkdir -p ~/.fonts
cp JetBrainsMono/*.ttf ~/.fonts/
# Update font cache
fc-cache -fv
```

Then select the **JetbrainsMono Nerd Font** font in your terminal settings.

Clone the `NvChad` config framework.

```bash
git clone https://github.com/NvChad/starter ~/.config/nvim
```

Then run `nvim`, run `:MasonInstallAll` command after lazy.nvim finishes downloading plugins.

Learn customization of ui & base46 from `:h nvui`.

`<Space>th`: Select the theme.

## `nvimTree`

Folder tree plugin.

`<C-n>`: Open the tree.

`<C-]>`: Cd into a directory.

`-`: Go up one directory level.

`<Enter>`: Open a file or directory.

`a, d, r`: Add, delete, rename folder or file.

## `Mason`

`:Mason`: Enter Mason UI.

`i`: Install.

`u`: Uninstall.

`X`: Update.

## `netrw`

The default file tree system in vim.

`%`: Create a file.

`d`: Create a directory.

`:Ex`: Exit the edit mode, then return to file tree.

## `packer`

Package manager.

`:PackerSync`: Synchronize the packages.

## `lazy`

Another package manager.

## `telescope`

File&string finder.

It is required to set the key remapping to use telescope.

## `treesitter`

What is this...

## `nvim-dap`


## Launch Script 


# Language Featured IDE

## C++

LSP: clangd

Linter: cpplint, cmakelint

DAP: codelldp

Formatter: clangd-format

TODO: Add config.

## Python

LSP: pyright

Linter: mypy, ruff

Formatter: black

DAP: dap, dap-python

TODO: Add config.



