# this file is part of the C64 Emulator Test Suite. public domain, no copyright

#.SILENT:

COMMONSRC = \
	common/target.inc \
	common/common.asm \
	common/print.asm \
	common/printhb.asm \
	common/showregs.asm \
	common/startup.asm \
	common/target.inc \
	common/waitborder.asm \
	common/waitkey.asm

ALLPROGS = \
	start.prg \
	ldab.prg \
	ldaz.prg \
	ldazx.prg \
	ldaa.prg \
	ldaax.prg \
	ldaay.prg \
	ldaix.prg \
	ldaiy.prg \
	staz.prg \
	stazx.prg \
	staa.prg \
	staax.prg \
	staay.prg \
	staix.prg \
	staiy.prg \
	ldxb.prg \
	ldxz.prg \
	ldxzy.prg \
	ldxa.prg \
	ldxay.prg \
	stxz.prg \
	stxzy.prg \
	stxa.prg \
	ldyb.prg \
	ldyz.prg \
	ldyzx.prg \
	ldya.prg \
	ldyax.prg \
	styz.prg \
	styzx.prg \
	stya.prg \
	taxn.prg \
	tayn.prg \
	txan.prg \
	tyan.prg \
	tsxn.prg \
	txsn.prg \
	phan.prg \
	plan.prg \
	phpn.prg \
	plpn.prg \
	inxn.prg \
	inyn.prg \
	dexn.prg \
	deyn.prg \
	incz.prg \
	inczx.prg \
	inca.prg \
	incax.prg \
	decz.prg \
	deczx.prg \
	deca.prg \
	decax.prg \
	asln.prg \
	aslz.prg \
	aslzx.prg \
	asla.prg \
	aslax.prg \
	lsrn.prg \
	lsrz.prg \
	lsrzx.prg \
	lsra.prg \
	lsrax.prg \
	roln.prg \
	rolz.prg \
	rolzx.prg \
	rola.prg \
	rolax.prg \
	rorn.prg \
	rorz.prg \
	rorzx.prg \
	rora.prg \
	rorax.prg \
	andb.prg \
	andz.prg \
	andzx.prg \
	anda.prg \
	andax.prg \
	anday.prg \
	andix.prg \
	andiy.prg \
	orab.prg \
	oraz.prg \
	orazx.prg \
	oraa.prg \
	oraax.prg \
	oraay.prg \
	oraix.prg \
	oraiy.prg \
	eorb.prg \
	eorz.prg \
	eorzx.prg \
	eora.prg \
	eorax.prg \
	eoray.prg \
	eorix.prg \
	eoriy.prg \
	clcn.prg \
	secn.prg \
	cldn.prg \
	sedn.prg \
	clin.prg \
	sein.prg \
	clvn.prg \
	adcb.prg \
	adcz.prg \
	adczx.prg \
	adca.prg \
	adcax.prg \
	adcay.prg \
	adcix.prg \
	adciy.prg \
	sbcb.prg \
	sbcz.prg \
	sbczx.prg \
	sbca.prg \
	sbcax.prg \
	sbcay.prg \
	sbcix.prg \
	sbciy.prg \
	cmpb.prg \
	cmpz.prg \
	cmpzx.prg \
	cmpa.prg \
	cmpax.prg \
	cmpay.prg \
	cmpix.prg \
	cmpiy.prg \
	cpxb.prg \
	cpxz.prg \
	cpxa.prg \
	cpyb.prg \
	cpyz.prg \
	cpya.prg \
	bitz.prg \
	bita.prg \
	brkn.prg \
	rtin.prg \
	jsrw.prg \
	rtsn.prg \
	jmpw.prg \
	jmpi.prg \
	beqr.prg \
	bner.prg \
	bmir.prg \
	bplr.prg \
	bcsr.prg \
	bccr.prg \
	bvsr.prg \
	bvcr.prg \
	nopn.prg \
	nopb.prg \
	nopz.prg \
	nopzx.prg \
	nopa.prg \
	nopax.prg \
	asoz.prg \
	asozx.prg \
	asoa.prg \
	asoax.prg \
	asoay.prg \
	asoix.prg \
	asoiy.prg \
	rlaz.prg \
	rlazx.prg \
	rlaa.prg \
	rlaax.prg \
	rlaay.prg \
	rlaix.prg \
	rlaiy.prg \
	lsez.prg \
	lsezx.prg \
	lsea.prg \
	lseax.prg \
	lseay.prg \
	lseix.prg \
	lseiy.prg \
	rraz.prg \
	rrazx.prg \
	rraa.prg \
	rraax.prg \
	rraay.prg \
	rraix.prg \
	rraiy.prg \
	dcmz.prg \
	dcmzx.prg \
	dcma.prg \
	dcmax.prg \
	dcmay.prg \
	dcmix.prg \
	dcmiy.prg \
	insz.prg \
	inszx.prg \
	insa.prg \
	insax.prg \
	insay.prg \
	insix.prg \
	insiy.prg \
	laxz.prg \
	laxzy.prg \
	laxa.prg \
	laxay.prg \
	laxix.prg \
	laxiy.prg \
	axsz.prg \
	axszy.prg \
	axsa.prg \
	axsix.prg \
	alrb.prg \
	arrb.prg \
	aneb.prg \
	lxab.prg \
	sbxb.prg \
	shaay.prg \
	shaiy.prg \
	shxay.prg \
	shyax.prg \
	shsay.prg \
	ancb.prg \
	lasay.prg \
	sbcb-eb.prg \
	trap1.prg \
	trap2.prg \
	trap3.prg \
	trap4.prg \
	trap5.prg \
	trap6.prg \
	trap7.prg \
	trap8.prg \
	trap9.prg \
	trap10.prg \
	trap11.prg \
	trap12.prg \
	trap13.prg \
	trap14.prg \
	trap15.prg \
	trap16.prg \
	trap17.prg \
	branchwrap.prg \
	cputiming.prg \
	irq.prg \
	nmi.prg \
	finish.prg \
	\
#	template.prg

ASFLAGS=-C -T -a -q -I ./common -D TARGET=0

all: $(ALLPROGS)

trap1.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=1 -i trap1-15.s -o trap1.prg
trap2.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=2 -i trap1-15.s -o trap2.prg
trap3.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=3 -i trap1-15.s -o trap3.prg
trap4.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=4 -i trap1-15.s -o trap4.prg
trap5.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=5 -i trap1-15.s -o trap5.prg
trap6.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=6 -i trap1-15.s -o trap6.prg
trap7.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=7 -i trap1-15.s -o trap7.prg
trap8.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=8 -i trap1-15.s -o trap8.prg
trap9.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=9 -i trap1-15.s -o trap9.prg
trap10.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=10 -i trap1-15.s -o trap10.prg
trap11.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=11 -i trap1-15.s -o trap11.prg
trap12.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=12 -i trap1-15.s -o trap12.prg
trap13.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=13 -i trap1-15.s -o trap13.prg
trap14.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=14 -i trap1-15.s -o trap14.prg
trap15.prg: trap1-15.s $(COMMONSRC)
	64tass $(ASFLAGS) -D TRAP=15 -i trap1-15.s -o trap15.prg

cputiming.prg: cputiming.s $(COMMONSRC)
	64tass $(ASFLAGS) -Wno-jmp-bug -i cputiming.s -o cputiming.prg

irq.prg: irq.s $(COMMONSRC)
	64tass $(ASFLAGS) -D NEWCIA=0 -i irq.s -o irq.prg
nmi.prg: nmi.s $(COMMONSRC)
	64tass $(ASFLAGS) -D NEWCIA=0 -i nmi.s -o nmi.prg

%.prg : %.s $(COMMONSRC)
	64tass $(ASFLAGS) -i "$*.s" -o "$*.prg"

clean:
	rm -f *.prg
