rm *.changes *.upload
cd usbso*
#make clean
rm debian/debhelper.log

#./debian/rules get-orig-source
#mv sdr*gz ..

#checksum not updated
debuild
debuild -S
#dpkg-buildpackage

cd ..
cd usbso*
rm debian/debhelper.log
cd ..

git commit -a -m "new build"
git push
exit

lintian --pedantic --all *dsc
lintian  *deb

for f in *changes; do
dput -l debexpo $f
dput -l ubuntu $f
done
