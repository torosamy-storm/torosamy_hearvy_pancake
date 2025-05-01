bin_directory=$(pwd)

cd $bin_directory




rm -rf out
mkdir out

cp -rf scripts ./out
cp -rf src ./out
cp -rf build.sh ./out
cp -rf package.sh ./out
cp -rf version.txt ./out

rm -rf ./out/src/torosamy_automatic_aiming/build/*
rm -rf ./out/src/torosamy_navigation/torosamy_navigation_client/build/*
rm -rf ./out/src/torosamy_serial_port/build/*



cd out



# 将当前目录下的所有内容压缩到上一级目录的 zip 文件中
tar -czvf "../torosamy_hearvy_pancake.tar.gz" .

cd ..
mv "torosamy_hearvy_pancake.tar.gz" out

