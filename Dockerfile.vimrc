

# install vimplug
RUN curl -fLo ~/.vim/autoload/plug.vim --create-dirs \
    https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim



# install the vimrc
COPY scripts/vimrc /root/.vimrc
RUN vim -E -s -u "$HOME/.vimrc" +PlugInstall +qall


